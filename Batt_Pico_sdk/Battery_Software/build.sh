#!/bin/bash

# Default values
NUM_JOBS=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 1)
OUTPUT_NAME="boot"  # Default output name

# Function to show usage
show_usage() {
    echo "Usage: $0 [-j jobs] [-n output_name]"
    echo "  -j: Number of parallel jobs for make (default: number of CPU cores)"
    echo "  -n: Name for the output .uf2 file (default: boot)"
    exit 1
}

# Parse command line arguments
while getopts "j:n:h" opt; do
    case ${opt} in
        j )
            NUM_JOBS=$OPTARG
            ;;
        n )
            OUTPUT_NAME=$OPTARG
            ;;
        h )
            show_usage
            ;;
        \? )
            show_usage
            ;;
    esac
done

# Print start message
echo "Starting Pico build process..."
echo "Using $NUM_JOBS parallel job(s)"
echo "Output will be named: ${OUTPUT_NAME}.uf2"

# Check if build directory exists, create if not
if [ ! -d "build" ]; then
    echo "Creating build directory..."
    mkdir build
fi

# Navigate to build directory
cd build || exit

# Clean build directory
echo "Cleaning build directory..."
rm -rf *

# Generate build files
echo "Generating build files with CMake..."
cmake ..

# Build the project with specified number of jobs
echo "Building project..."
make -j"$NUM_JOBS"

# Check if build was successful
if [ $? -eq 0 ]; then
    echo "Build successful!"
    
    # Check if boot.uf2 was created
    if [ -f "Battery_Software/boot/boot.uf2" ]; then
        echo "UF2 file created successfully at: Battery_Software/boot/boot.uf2"
        
        # Copy and rename the UF2 file to root directory
        cp "Battery_Software/boot/boot.uf2" "../${OUTPUT_NAME}.uf2"
        if [ $? -eq 0 ]; then
            echo "UF2 file copied to root directory as: ${OUTPUT_NAME}.uf2"
        else
            echo "Error: Failed to copy UF2 file to root directory"
            exit 1
        fi
    else
        echo "Error: UF2 file not found"
        exit 1
    fi
else
    echo "Build failed!"
    exit 1
fi

# Return to original directory
cd ..

echo "Build process complete!"