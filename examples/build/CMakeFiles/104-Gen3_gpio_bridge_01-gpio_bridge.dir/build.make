# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Produce verbose output by default.
VERBOSE = 1

# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zhou/kortex/api_cpp/examples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zhou/kortex/api_cpp/examples/build

# Include any dependencies generated for this target.
include CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/flags.make

CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/104-Gen3_gpio_bridge/01-gpio_bridge.cpp.o: CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/flags.make
CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/104-Gen3_gpio_bridge/01-gpio_bridge.cpp.o: ../104-Gen3_gpio_bridge/01-gpio_bridge.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhou/kortex/api_cpp/examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/104-Gen3_gpio_bridge/01-gpio_bridge.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/104-Gen3_gpio_bridge/01-gpio_bridge.cpp.o -c /home/zhou/kortex/api_cpp/examples/104-Gen3_gpio_bridge/01-gpio_bridge.cpp

CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/104-Gen3_gpio_bridge/01-gpio_bridge.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/104-Gen3_gpio_bridge/01-gpio_bridge.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhou/kortex/api_cpp/examples/104-Gen3_gpio_bridge/01-gpio_bridge.cpp > CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/104-Gen3_gpio_bridge/01-gpio_bridge.cpp.i

CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/104-Gen3_gpio_bridge/01-gpio_bridge.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/104-Gen3_gpio_bridge/01-gpio_bridge.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhou/kortex/api_cpp/examples/104-Gen3_gpio_bridge/01-gpio_bridge.cpp -o CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/104-Gen3_gpio_bridge/01-gpio_bridge.cpp.s

CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/104-Gen3_gpio_bridge/01-gpio_bridge.cpp.o.requires:

.PHONY : CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/104-Gen3_gpio_bridge/01-gpio_bridge.cpp.o.requires

CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/104-Gen3_gpio_bridge/01-gpio_bridge.cpp.o.provides: CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/104-Gen3_gpio_bridge/01-gpio_bridge.cpp.o.requires
	$(MAKE) -f CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/build.make CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/104-Gen3_gpio_bridge/01-gpio_bridge.cpp.o.provides.build
.PHONY : CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/104-Gen3_gpio_bridge/01-gpio_bridge.cpp.o.provides

CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/104-Gen3_gpio_bridge/01-gpio_bridge.cpp.o.provides.build: CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/104-Gen3_gpio_bridge/01-gpio_bridge.cpp.o


# Object files for target 104-Gen3_gpio_bridge_01-gpio_bridge
104__Gen3_gpio_bridge_01__gpio_bridge_OBJECTS = \
"CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/104-Gen3_gpio_bridge/01-gpio_bridge.cpp.o"

# External object files for target 104-Gen3_gpio_bridge_01-gpio_bridge
104__Gen3_gpio_bridge_01__gpio_bridge_EXTERNAL_OBJECTS =

104-Gen3_gpio_bridge_01-gpio_bridge: CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/104-Gen3_gpio_bridge/01-gpio_bridge.cpp.o
104-Gen3_gpio_bridge_01-gpio_bridge: CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/build.make
104-Gen3_gpio_bridge_01-gpio_bridge: ../kortex_api/lib/release/libKortexApiCpp.a
104-Gen3_gpio_bridge_01-gpio_bridge: CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zhou/kortex/api_cpp/examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable 104-Gen3_gpio_bridge_01-gpio_bridge"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/build: 104-Gen3_gpio_bridge_01-gpio_bridge

.PHONY : CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/build

CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/requires: CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/104-Gen3_gpio_bridge/01-gpio_bridge.cpp.o.requires

.PHONY : CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/requires

CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/cmake_clean.cmake
.PHONY : CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/clean

CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/depend:
	cd /home/zhou/kortex/api_cpp/examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zhou/kortex/api_cpp/examples /home/zhou/kortex/api_cpp/examples /home/zhou/kortex/api_cpp/examples/build /home/zhou/kortex/api_cpp/examples/build /home/zhou/kortex/api_cpp/examples/build/CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/104-Gen3_gpio_bridge_01-gpio_bridge.dir/depend

