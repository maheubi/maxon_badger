# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


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
CMAKE_SOURCE_DIR = /home/user/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/user/catkin_ws/build

# Include any dependencies generated for this target.
include object_detection/CMakeFiles/simple_layer.dir/depend.make

# Include the progress variables for this target.
include object_detection/CMakeFiles/simple_layer.dir/progress.make

# Include the compile flags for this target's objects.
include object_detection/CMakeFiles/simple_layer.dir/flags.make

object_detection/CMakeFiles/simple_layer.dir/src/simple_layer.cpp.o: object_detection/CMakeFiles/simple_layer.dir/flags.make
object_detection/CMakeFiles/simple_layer.dir/src/simple_layer.cpp.o: /home/user/catkin_ws/src/object_detection/src/simple_layer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object object_detection/CMakeFiles/simple_layer.dir/src/simple_layer.cpp.o"
	cd /home/user/catkin_ws/build/object_detection && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simple_layer.dir/src/simple_layer.cpp.o -c /home/user/catkin_ws/src/object_detection/src/simple_layer.cpp

object_detection/CMakeFiles/simple_layer.dir/src/simple_layer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simple_layer.dir/src/simple_layer.cpp.i"
	cd /home/user/catkin_ws/build/object_detection && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/catkin_ws/src/object_detection/src/simple_layer.cpp > CMakeFiles/simple_layer.dir/src/simple_layer.cpp.i

object_detection/CMakeFiles/simple_layer.dir/src/simple_layer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simple_layer.dir/src/simple_layer.cpp.s"
	cd /home/user/catkin_ws/build/object_detection && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/catkin_ws/src/object_detection/src/simple_layer.cpp -o CMakeFiles/simple_layer.dir/src/simple_layer.cpp.s

# Object files for target simple_layer
simple_layer_OBJECTS = \
"CMakeFiles/simple_layer.dir/src/simple_layer.cpp.o"

# External object files for target simple_layer
simple_layer_EXTERNAL_OBJECTS =

/home/user/catkin_ws/devel/lib/libsimple_layer.so: object_detection/CMakeFiles/simple_layer.dir/src/simple_layer.cpp.o
/home/user/catkin_ws/devel/lib/libsimple_layer.so: object_detection/CMakeFiles/simple_layer.dir/build.make
/home/user/catkin_ws/devel/lib/libsimple_layer.so: object_detection/CMakeFiles/simple_layer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/user/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/user/catkin_ws/devel/lib/libsimple_layer.so"
	cd /home/user/catkin_ws/build/object_detection && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simple_layer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
object_detection/CMakeFiles/simple_layer.dir/build: /home/user/catkin_ws/devel/lib/libsimple_layer.so

.PHONY : object_detection/CMakeFiles/simple_layer.dir/build

object_detection/CMakeFiles/simple_layer.dir/clean:
	cd /home/user/catkin_ws/build/object_detection && $(CMAKE_COMMAND) -P CMakeFiles/simple_layer.dir/cmake_clean.cmake
.PHONY : object_detection/CMakeFiles/simple_layer.dir/clean

object_detection/CMakeFiles/simple_layer.dir/depend:
	cd /home/user/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/catkin_ws/src /home/user/catkin_ws/src/object_detection /home/user/catkin_ws/build /home/user/catkin_ws/build/object_detection /home/user/catkin_ws/build/object_detection/CMakeFiles/simple_layer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : object_detection/CMakeFiles/simple_layer.dir/depend

