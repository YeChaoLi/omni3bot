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

# Utility rule file for dfu-flash.

# Include any custom commands dependencies for this target.
include CMakeFiles/dfu-flash.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/dfu-flash.dir/progress.make

CMakeFiles/dfu-flash:
	/usr/bin/cmake -D ESP_DFU_BIN="/home/chao/Workspace/Hackathon/omni3bot/build/dfu.bin" -D ESP_DFU_PID="9" -P /home/chao/esp/master/esp-idf/tools/cmake/run_dfu_util.cmake

dfu-flash: CMakeFiles/dfu-flash
dfu-flash: CMakeFiles/dfu-flash.dir/build.make
.PHONY : dfu-flash

# Rule to build all files generated by this target.
CMakeFiles/dfu-flash.dir/build: dfu-flash
.PHONY : CMakeFiles/dfu-flash.dir/build

CMakeFiles/dfu-flash.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/dfu-flash.dir/cmake_clean.cmake
.PHONY : CMakeFiles/dfu-flash.dir/clean

CMakeFiles/dfu-flash.dir/depend:
	cd /home/chao/Workspace/Hackathon/omni3bot/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chao/Workspace/Hackathon/omni3bot /home/chao/Workspace/Hackathon/omni3bot /home/chao/Workspace/Hackathon/omni3bot/build /home/chao/Workspace/Hackathon/omni3bot/build /home/chao/Workspace/Hackathon/omni3bot/build/CMakeFiles/dfu-flash.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/dfu-flash.dir/depend

