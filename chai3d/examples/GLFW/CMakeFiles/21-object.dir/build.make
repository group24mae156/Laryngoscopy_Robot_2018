# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/michael/chai3d

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/michael/chai3d

# Include any dependencies generated for this target.
include examples/GLFW/CMakeFiles/21-object.dir/depend.make

# Include the progress variables for this target.
include examples/GLFW/CMakeFiles/21-object.dir/progress.make

# Include the compile flags for this target's objects.
include examples/GLFW/CMakeFiles/21-object.dir/flags.make

examples/GLFW/CMakeFiles/21-object.dir/21-object/21-object.cpp.o: examples/GLFW/CMakeFiles/21-object.dir/flags.make
examples/GLFW/CMakeFiles/21-object.dir/21-object/21-object.cpp.o: examples/GLFW/21-object/21-object.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/michael/chai3d/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/GLFW/CMakeFiles/21-object.dir/21-object/21-object.cpp.o"
	cd /home/michael/chai3d/examples/GLFW && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/21-object.dir/21-object/21-object.cpp.o -c /home/michael/chai3d/examples/GLFW/21-object/21-object.cpp

examples/GLFW/CMakeFiles/21-object.dir/21-object/21-object.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/21-object.dir/21-object/21-object.cpp.i"
	cd /home/michael/chai3d/examples/GLFW && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/michael/chai3d/examples/GLFW/21-object/21-object.cpp > CMakeFiles/21-object.dir/21-object/21-object.cpp.i

examples/GLFW/CMakeFiles/21-object.dir/21-object/21-object.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/21-object.dir/21-object/21-object.cpp.s"
	cd /home/michael/chai3d/examples/GLFW && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/michael/chai3d/examples/GLFW/21-object/21-object.cpp -o CMakeFiles/21-object.dir/21-object/21-object.cpp.s

examples/GLFW/CMakeFiles/21-object.dir/21-object/21-object.cpp.o.requires:

.PHONY : examples/GLFW/CMakeFiles/21-object.dir/21-object/21-object.cpp.o.requires

examples/GLFW/CMakeFiles/21-object.dir/21-object/21-object.cpp.o.provides: examples/GLFW/CMakeFiles/21-object.dir/21-object/21-object.cpp.o.requires
	$(MAKE) -f examples/GLFW/CMakeFiles/21-object.dir/build.make examples/GLFW/CMakeFiles/21-object.dir/21-object/21-object.cpp.o.provides.build
.PHONY : examples/GLFW/CMakeFiles/21-object.dir/21-object/21-object.cpp.o.provides

examples/GLFW/CMakeFiles/21-object.dir/21-object/21-object.cpp.o.provides.build: examples/GLFW/CMakeFiles/21-object.dir/21-object/21-object.cpp.o


# Object files for target 21-object
21__object_OBJECTS = \
"CMakeFiles/21-object.dir/21-object/21-object.cpp.o"

# External object files for target 21-object
21__object_EXTERNAL_OBJECTS =

bin/lin-x86_64/21-object: examples/GLFW/CMakeFiles/21-object.dir/21-object/21-object.cpp.o
bin/lin-x86_64/21-object: examples/GLFW/CMakeFiles/21-object.dir/build.make
bin/lin-x86_64/21-object: libchai3d.a
bin/lin-x86_64/21-object: /usr/lib/x86_64-linux-gnu/libGLU.so
bin/lin-x86_64/21-object: /usr/lib/x86_64-linux-gnu/libGL.so
bin/lin-x86_64/21-object: extras/GLFW/libglfw.a
bin/lin-x86_64/21-object: /usr/lib/x86_64-linux-gnu/libGLU.so
bin/lin-x86_64/21-object: /usr/lib/x86_64-linux-gnu/libGL.so
bin/lin-x86_64/21-object: /usr/lib/x86_64-linux-gnu/libGLU.so
bin/lin-x86_64/21-object: /usr/lib/x86_64-linux-gnu/libGL.so
bin/lin-x86_64/21-object: examples/GLFW/CMakeFiles/21-object.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/michael/chai3d/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/lin-x86_64/21-object"
	cd /home/michael/chai3d/examples/GLFW && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/21-object.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/GLFW/CMakeFiles/21-object.dir/build: bin/lin-x86_64/21-object

.PHONY : examples/GLFW/CMakeFiles/21-object.dir/build

examples/GLFW/CMakeFiles/21-object.dir/requires: examples/GLFW/CMakeFiles/21-object.dir/21-object/21-object.cpp.o.requires

.PHONY : examples/GLFW/CMakeFiles/21-object.dir/requires

examples/GLFW/CMakeFiles/21-object.dir/clean:
	cd /home/michael/chai3d/examples/GLFW && $(CMAKE_COMMAND) -P CMakeFiles/21-object.dir/cmake_clean.cmake
.PHONY : examples/GLFW/CMakeFiles/21-object.dir/clean

examples/GLFW/CMakeFiles/21-object.dir/depend:
	cd /home/michael/chai3d && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/michael/chai3d /home/michael/chai3d/examples/GLFW /home/michael/chai3d /home/michael/chai3d/examples/GLFW /home/michael/chai3d/examples/GLFW/CMakeFiles/21-object.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/GLFW/CMakeFiles/21-object.dir/depend

