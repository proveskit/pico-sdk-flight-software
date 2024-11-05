# pico-sdk-flight-software
This is a separate repo for software developed using the Pico SDK. 


# Firmware Build Process
*Written by Claude 3.5*
## Prerequisites
- CMake
- Ninja build system
- Raspberry Pi Pico SDK (expected at ~/.pico-sdk/sdk/2.0.0)
- ARM GCC Toolchain

## Build Script Usage
The project includes a `big_build.sh` script that simplifies the build process. The script supports several options:

```bash
./build.sh [-j jobs] [-n output_name] [-c] [-v] [-h]

Options:
 -j: Number of parallel jobs for compilation (default: number of CPU cores)
 -n: Name for the output .uf2 file (default: boot)
 -c: Clean only, don't build
 -v: Verbose output
 -h: Show help message

Example:
 ./big_build.sh -j8 -n battery_firm_v1.0 -v
```

## Build Process Overview
1. The script first checks for required tools (CMake, Ninja)
2. Cleans the build directory to ensure a fresh build
3. Generates build files using CMake with Ninja generator
4. Compiles the project using specified number of parallel jobs
5. Generates the UF2 file for Pico flashing
6. Creates a backup of any previous builds
7. Copies the final UF2 file to the project root directory

## Project Structure
```
Batt_Pico_sdk/
├── Battery_Software/
│   ├── lib/           # Core libraries and functionality
│   └── boot/          # Boot and initialization code
├── build/             # Build directory (generated)
└── build.sh           # Build script
```

## Output Files
- Primary output: `<name>.uf2` in project root
- Backup files: `<name>_backup_<timestamp>.uf2`
- Build artifacts in `build/` directory

## Flashing the Firmware
1. Hold the BOOTSEL button on the Pico
2. Connect the Pico to your computer via USB
3. Release BOOTSEL once the Pico mounts as a drive
4. Copy the generated .uf2 file to the mounted Pico drive
5. The Pico will automatically reboot and run the new firmware

## Version Control
The script automatically includes timestamps in backup files and can handle version numbering through the output name parameter. It's recommended to use semantic versioning in the output name:
```bash
./big_build.sh -n battery_firm_v1.0.0
```