# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/chao/Workspace/Hackathon/omni3bot

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/chao/Workspace/Hackathon/omni3bot/build

# Include any dependencies generated for this target.
include esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/compiler_depend.make

# Include the progress variables for this target.
include esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/progress.make

# Include the compile flags for this target's objects.
include esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/flags.make

esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/library/everest.c.obj: esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/flags.make
esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/library/everest.c.obj: /home/chao/esp/master/esp-idf/components/mbedtls/mbedtls/3rdparty/everest/library/everest.c
esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/library/everest.c.obj: esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/chao/Workspace/Hackathon/omni3bot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/library/everest.c.obj"
	cd /home/chao/Workspace/Hackathon/omni3bot/build/esp-idf/mbedtls/mbedtls/3rdparty/everest && /home/chao/.espressif/tools/xtensa-esp-elf/esp-15.1.0_20250607/xtensa-esp-elf/bin/xtensa-esp32s3-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/library/everest.c.obj -MF CMakeFiles/everest.dir/library/everest.c.obj.d -o CMakeFiles/everest.dir/library/everest.c.obj -c /home/chao/esp/master/esp-idf/components/mbedtls/mbedtls/3rdparty/everest/library/everest.c

esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/library/everest.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/everest.dir/library/everest.c.i"
	cd /home/chao/Workspace/Hackathon/omni3bot/build/esp-idf/mbedtls/mbedtls/3rdparty/everest && /home/chao/.espressif/tools/xtensa-esp-elf/esp-15.1.0_20250607/xtensa-esp-elf/bin/xtensa-esp32s3-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/chao/esp/master/esp-idf/components/mbedtls/mbedtls/3rdparty/everest/library/everest.c > CMakeFiles/everest.dir/library/everest.c.i

esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/library/everest.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/everest.dir/library/everest.c.s"
	cd /home/chao/Workspace/Hackathon/omni3bot/build/esp-idf/mbedtls/mbedtls/3rdparty/everest && /home/chao/.espressif/tools/xtensa-esp-elf/esp-15.1.0_20250607/xtensa-esp-elf/bin/xtensa-esp32s3-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/chao/esp/master/esp-idf/components/mbedtls/mbedtls/3rdparty/everest/library/everest.c -o CMakeFiles/everest.dir/library/everest.c.s

esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/library/x25519.c.obj: esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/flags.make
esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/library/x25519.c.obj: /home/chao/esp/master/esp-idf/components/mbedtls/mbedtls/3rdparty/everest/library/x25519.c
esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/library/x25519.c.obj: esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/chao/Workspace/Hackathon/omni3bot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/library/x25519.c.obj"
	cd /home/chao/Workspace/Hackathon/omni3bot/build/esp-idf/mbedtls/mbedtls/3rdparty/everest && /home/chao/.espressif/tools/xtensa-esp-elf/esp-15.1.0_20250607/xtensa-esp-elf/bin/xtensa-esp32s3-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/library/x25519.c.obj -MF CMakeFiles/everest.dir/library/x25519.c.obj.d -o CMakeFiles/everest.dir/library/x25519.c.obj -c /home/chao/esp/master/esp-idf/components/mbedtls/mbedtls/3rdparty/everest/library/x25519.c

esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/library/x25519.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/everest.dir/library/x25519.c.i"
	cd /home/chao/Workspace/Hackathon/omni3bot/build/esp-idf/mbedtls/mbedtls/3rdparty/everest && /home/chao/.espressif/tools/xtensa-esp-elf/esp-15.1.0_20250607/xtensa-esp-elf/bin/xtensa-esp32s3-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/chao/esp/master/esp-idf/components/mbedtls/mbedtls/3rdparty/everest/library/x25519.c > CMakeFiles/everest.dir/library/x25519.c.i

esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/library/x25519.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/everest.dir/library/x25519.c.s"
	cd /home/chao/Workspace/Hackathon/omni3bot/build/esp-idf/mbedtls/mbedtls/3rdparty/everest && /home/chao/.espressif/tools/xtensa-esp-elf/esp-15.1.0_20250607/xtensa-esp-elf/bin/xtensa-esp32s3-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/chao/esp/master/esp-idf/components/mbedtls/mbedtls/3rdparty/everest/library/x25519.c -o CMakeFiles/everest.dir/library/x25519.c.s

esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/library/Hacl_Curve25519_joined.c.obj: esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/flags.make
esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/library/Hacl_Curve25519_joined.c.obj: /home/chao/esp/master/esp-idf/components/mbedtls/mbedtls/3rdparty/everest/library/Hacl_Curve25519_joined.c
esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/library/Hacl_Curve25519_joined.c.obj: esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/chao/Workspace/Hackathon/omni3bot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/library/Hacl_Curve25519_joined.c.obj"
	cd /home/chao/Workspace/Hackathon/omni3bot/build/esp-idf/mbedtls/mbedtls/3rdparty/everest && /home/chao/.espressif/tools/xtensa-esp-elf/esp-15.1.0_20250607/xtensa-esp-elf/bin/xtensa-esp32s3-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/library/Hacl_Curve25519_joined.c.obj -MF CMakeFiles/everest.dir/library/Hacl_Curve25519_joined.c.obj.d -o CMakeFiles/everest.dir/library/Hacl_Curve25519_joined.c.obj -c /home/chao/esp/master/esp-idf/components/mbedtls/mbedtls/3rdparty/everest/library/Hacl_Curve25519_joined.c

esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/library/Hacl_Curve25519_joined.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/everest.dir/library/Hacl_Curve25519_joined.c.i"
	cd /home/chao/Workspace/Hackathon/omni3bot/build/esp-idf/mbedtls/mbedtls/3rdparty/everest && /home/chao/.espressif/tools/xtensa-esp-elf/esp-15.1.0_20250607/xtensa-esp-elf/bin/xtensa-esp32s3-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/chao/esp/master/esp-idf/components/mbedtls/mbedtls/3rdparty/everest/library/Hacl_Curve25519_joined.c > CMakeFiles/everest.dir/library/Hacl_Curve25519_joined.c.i

esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/library/Hacl_Curve25519_joined.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/everest.dir/library/Hacl_Curve25519_joined.c.s"
	cd /home/chao/Workspace/Hackathon/omni3bot/build/esp-idf/mbedtls/mbedtls/3rdparty/everest && /home/chao/.espressif/tools/xtensa-esp-elf/esp-15.1.0_20250607/xtensa-esp-elf/bin/xtensa-esp32s3-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/chao/esp/master/esp-idf/components/mbedtls/mbedtls/3rdparty/everest/library/Hacl_Curve25519_joined.c -o CMakeFiles/everest.dir/library/Hacl_Curve25519_joined.c.s

# Object files for target everest
everest_OBJECTS = \
"CMakeFiles/everest.dir/library/everest.c.obj" \
"CMakeFiles/everest.dir/library/x25519.c.obj" \
"CMakeFiles/everest.dir/library/Hacl_Curve25519_joined.c.obj"

# External object files for target everest
everest_EXTERNAL_OBJECTS =

esp-idf/mbedtls/mbedtls/3rdparty/everest/libeverest.a: esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/library/everest.c.obj
esp-idf/mbedtls/mbedtls/3rdparty/everest/libeverest.a: esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/library/x25519.c.obj
esp-idf/mbedtls/mbedtls/3rdparty/everest/libeverest.a: esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/library/Hacl_Curve25519_joined.c.obj
esp-idf/mbedtls/mbedtls/3rdparty/everest/libeverest.a: esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/build.make
esp-idf/mbedtls/mbedtls/3rdparty/everest/libeverest.a: esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/chao/Workspace/Hackathon/omni3bot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library libeverest.a"
	cd /home/chao/Workspace/Hackathon/omni3bot/build/esp-idf/mbedtls/mbedtls/3rdparty/everest && $(CMAKE_COMMAND) -P CMakeFiles/everest.dir/cmake_clean_target.cmake
	cd /home/chao/Workspace/Hackathon/omni3bot/build/esp-idf/mbedtls/mbedtls/3rdparty/everest && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/everest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/build: esp-idf/mbedtls/mbedtls/3rdparty/everest/libeverest.a
.PHONY : esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/build

esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/clean:
	cd /home/chao/Workspace/Hackathon/omni3bot/build/esp-idf/mbedtls/mbedtls/3rdparty/everest && $(CMAKE_COMMAND) -P CMakeFiles/everest.dir/cmake_clean.cmake
.PHONY : esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/clean

esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/depend:
	cd /home/chao/Workspace/Hackathon/omni3bot/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chao/Workspace/Hackathon/omni3bot /home/chao/esp/master/esp-idf/components/mbedtls/mbedtls/3rdparty/everest /home/chao/Workspace/Hackathon/omni3bot/build /home/chao/Workspace/Hackathon/omni3bot/build/esp-idf/mbedtls/mbedtls/3rdparty/everest /home/chao/Workspace/Hackathon/omni3bot/build/esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : esp-idf/mbedtls/mbedtls/3rdparty/everest/CMakeFiles/everest.dir/depend

