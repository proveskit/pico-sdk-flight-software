# Written with Claude 3.5

#!/bin/bash

# Default values and constants
NUM_JOBS=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 1)
OUTPUT_NAME="boot"  # Default output name
BUILD_DIR="build"
CLEAN_ONLY=0
VERBOSE=0
BUILD_TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
VERSION=${VERSION:-"0.1.0"}  # Default version if not set

# Function to show usage
show_usage() {
    echo "Usage: $0 [-j jobs] [-n output_name] [-c] [-v] [-h]"
    echo "Options:"
    echo "  -j: Number of parallel jobs for make (default: number of CPU cores)"
    echo "  -n: Name for the output .uf2 file (default: boot)"
    echo "  -c: Clean only, don't build"
    echo "  -v: Verbose output"
    echo "  -h: Show this help message"
    echo
    echo "Example:"
    echo "  $0 -j8 -n battery_firm_v1.0 -v"
    exit 1
}

# Function to clean build directory
clean_build() {
    echo "Cleaning build directory..."
    if [ -d "$BUILD_DIR" ]; then
        # Remove everything in the build directory
        rm -rf "$BUILD_DIR"
        # Recreate the empty directory
        mkdir "$BUILD_DIR"
        echo "Build directory cleaned and recreated"
    else
        echo "Creating new build directory..."
        mkdir "$BUILD_DIR"
    fi
}

# Function to check requirements
check_requirements() {
    local missing_tools=0
    
    echo "Checking build requirements..."
    
    if ! command -v cmake >/dev/null 2>&1; then
        echo "Error: cmake not found"
        missing_tools=1
    fi
    
    if ! command -v ninja >/dev/null 2>&1; then
        echo "Error: ninja not found"
        missing_tools=1
    fi
    
    # Check for Pico SDK in the default location
    if [ ! -d "$HOME/.pico-sdk/sdk/2.0.0" ]; then
        echo "Warning: Pico SDK not found in default location ($HOME/.pico-sdk/sdk/2.0.0)"
        echo "Build will continue using CMake-configured SDK path"
    else
        echo "Found Pico SDK in: $HOME/.pico-sdk/sdk/2.0.0"
    fi
    
    if [ $missing_tools -eq 1 ]; then
        echo "Please install missing requirements and try again"
        exit 1
    fi
    
    echo "All requirements satisfied"
}

# Parse command line arguments
while getopts "j:n:cvh" opt; do
    case ${opt} in
        j )
            NUM_JOBS=$OPTARG
            ;;
        n )
            OUTPUT_NAME=$OPTARG
            ;;
        c )
            CLEAN_ONLY=1
            ;;
        v )
            VERBOSE=1
            ;;
        h )
            show_usage
            ;;
        \? )
            show_usage
            ;;
    esac
done

# Handle clean-only option
if [ $CLEAN_ONLY -eq 1 ]; then
    clean_build
    exit 0
fi

# Set build options based on verbose flag
if [ $VERBOSE -eq 1 ]; then
    CMAKE_OPTS="-DCMAKE_VERBOSE_MAKEFILE:BOOL=ON"
    NINJA_OPTS="-v"
else
    CMAKE_OPTS=""
    NINJA_OPTS=""
fi

# Print start message
echo "=== Pico Build Script ==="
echo "Build timestamp: $BUILD_TIMESTAMP"
echo "Version: $VERSION"
echo "Using $NUM_JOBS parallel job(s)"
echo "Output will be named: ${OUTPUT_NAME}.uf2"
if [ $VERBOSE -eq 1 ]; then
    echo "Verbose output enabled"
fi
echo "======================="

# Check requirements before proceeding
check_requirements

# Check if build directory exists, create if not
if [ ! -d "$BUILD_DIR" ]; then
    echo "Creating build directory..."
    mkdir "$BUILD_DIR"
fi

# Navigate to build directory
cd "$BUILD_DIR" || exit 1

# Clean build directory
clean_build

# Generate build files
echo "Generating build files with CMake..."
if ! cmake -G Ninja $CMAKE_OPTS ..; then
    echo "CMake configuration failed!"
    exit 1
fi

# Build the project with specified number of jobs
echo "Building project..."
if ! ninja $NINJA_OPTS -j"$NUM_JOBS"; then
    echo "Build failed!"
    exit 1
fi

# Handle the output file
if [ -f "Battery_Software/boot/boot.uf2" ]; then
    echo "UF2 file created successfully at: Battery_Software/boot/boot.uf2"
    
    # Backup previous build if it exists
    if [ -f "../${OUTPUT_NAME}.uf2" ]; then
        echo "Backing up previous build..."
        mv "../${OUTPUT_NAME}.uf2" "../${OUTPUT_NAME}_backup_${BUILD_TIMESTAMP}.uf2"
        echo "Previous build backed up as: ${OUTPUT_NAME}_backup_${BUILD_TIMESTAMP}.uf2"
    fi
    
    # Copy and rename the UF2 file to root directory
    if cp "Battery_Software/boot/boot.uf2" "../${OUTPUT_NAME}.uf2"; then
        echo "UF2 file copied to root directory as: ${OUTPUT_NAME}.uf2"
    else
        echo "Error: Failed to copy UF2 file to root directory"
        exit 1
    fi
else
    echo "Error: UF2 file not found"
    exit 1
fi

# Return to original directory
cd ..

echo "Build process complete!"
echo "Output file: ${OUTPUT_NAME}.uf2"
if [ -f "${OUTPUT_NAME}_backup_${BUILD_TIMESTAMP}.uf2" ]; then
    echo "Backup file: ${OUTPUT_NAME}_backup_${BUILD_TIMESTAMP}.uf2"
fi