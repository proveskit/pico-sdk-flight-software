# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.30

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.30.5/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.30.5/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/mattchang/pico-sdk/pico-sdk/burn-test/Batt_Pico_sdk

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/mattchang/pico-sdk/pico-sdk/burn-test/Batt_Pico_sdk/test

# Utility rule file for boot_blink_pio_h.

# Include any custom commands dependencies for this target.
include Battery_Software/boot/CMakeFiles/boot_blink_pio_h.dir/compiler_depend.make

# Include the progress variables for this target.
include Battery_Software/boot/CMakeFiles/boot_blink_pio_h.dir/progress.make

Battery_Software/boot/CMakeFiles/boot_blink_pio_h: Battery_Software/boot/blink.pio.h

Battery_Software/boot/blink.pio.h: /Users/mattchang/pico-sdk/pico-sdk/burn-test/Batt_Pico_sdk/Battery_Software/boot/blink.pio
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/mattchang/pico-sdk/pico-sdk/burn-test/Batt_Pico_sdk/test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating blink.pio.h"
	cd /Users/mattchang/pico-sdk/pico-sdk/burn-test/Batt_Pico_sdk/test/Battery_Software/boot && ../../pioasm/pioasm -o c-sdk /Users/mattchang/pico-sdk/pico-sdk/burn-test/Batt_Pico_sdk/Battery_Software/boot/blink.pio /Users/mattchang/pico-sdk/pico-sdk/burn-test/Batt_Pico_sdk/test/Battery_Software/boot/blink.pio.h

boot_blink_pio_h: Battery_Software/boot/CMakeFiles/boot_blink_pio_h
boot_blink_pio_h: Battery_Software/boot/blink.pio.h
boot_blink_pio_h: Battery_Software/boot/CMakeFiles/boot_blink_pio_h.dir/build.make
.PHONY : boot_blink_pio_h

# Rule to build all files generated by this target.
Battery_Software/boot/CMakeFiles/boot_blink_pio_h.dir/build: boot_blink_pio_h
.PHONY : Battery_Software/boot/CMakeFiles/boot_blink_pio_h.dir/build

Battery_Software/boot/CMakeFiles/boot_blink_pio_h.dir/clean:
	cd /Users/mattchang/pico-sdk/pico-sdk/burn-test/Batt_Pico_sdk/test/Battery_Software/boot && $(CMAKE_COMMAND) -P CMakeFiles/boot_blink_pio_h.dir/cmake_clean.cmake
.PHONY : Battery_Software/boot/CMakeFiles/boot_blink_pio_h.dir/clean

Battery_Software/boot/CMakeFiles/boot_blink_pio_h.dir/depend:
	cd /Users/mattchang/pico-sdk/pico-sdk/burn-test/Batt_Pico_sdk/test && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/mattchang/pico-sdk/pico-sdk/burn-test/Batt_Pico_sdk /Users/mattchang/pico-sdk/pico-sdk/burn-test/Batt_Pico_sdk/Battery_Software/boot /Users/mattchang/pico-sdk/pico-sdk/burn-test/Batt_Pico_sdk/test /Users/mattchang/pico-sdk/pico-sdk/burn-test/Batt_Pico_sdk/test/Battery_Software/boot /Users/mattchang/pico-sdk/pico-sdk/burn-test/Batt_Pico_sdk/test/Battery_Software/boot/CMakeFiles/boot_blink_pio_h.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : Battery_Software/boot/CMakeFiles/boot_blink_pio_h.dir/depend

