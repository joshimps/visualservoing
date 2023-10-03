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
CMAKE_SOURCE_DIR = /home/joshimps/git/visualservoing

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/joshimps/git/visualservoing/build

# Include any dependencies generated for this target.
include CMakeFiles/visual_servoing_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/visual_servoing_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/visual_servoing_test.dir/flags.make

CMakeFiles/visual_servoing_test.dir/test/utest.cpp.o: CMakeFiles/visual_servoing_test.dir/flags.make
CMakeFiles/visual_servoing_test.dir/test/utest.cpp.o: ../test/utest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/joshimps/git/visualservoing/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/visual_servoing_test.dir/test/utest.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/visual_servoing_test.dir/test/utest.cpp.o -c /home/joshimps/git/visualservoing/test/utest.cpp

CMakeFiles/visual_servoing_test.dir/test/utest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/visual_servoing_test.dir/test/utest.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/joshimps/git/visualservoing/test/utest.cpp > CMakeFiles/visual_servoing_test.dir/test/utest.cpp.i

CMakeFiles/visual_servoing_test.dir/test/utest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/visual_servoing_test.dir/test/utest.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/joshimps/git/visualservoing/test/utest.cpp -o CMakeFiles/visual_servoing_test.dir/test/utest.cpp.s

# Object files for target visual_servoing_test
visual_servoing_test_OBJECTS = \
"CMakeFiles/visual_servoing_test.dir/test/utest.cpp.o"

# External object files for target visual_servoing_test
visual_servoing_test_EXTERNAL_OBJECTS =

devel/lib/visual_servoing/visual_servoing_test: CMakeFiles/visual_servoing_test.dir/test/utest.cpp.o
devel/lib/visual_servoing/visual_servoing_test: CMakeFiles/visual_servoing_test.dir/build.make
devel/lib/visual_servoing/visual_servoing_test: gtest/lib/libgtestd.so
devel/lib/visual_servoing/visual_servoing_test: devel/lib/libvisual_servoing_robot.so
devel/lib/visual_servoing/visual_servoing_test: devel/lib/libvisual_servoing_robotController.so
devel/lib/visual_servoing/visual_servoing_test: CMakeFiles/visual_servoing_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/joshimps/git/visualservoing/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable devel/lib/visual_servoing/visual_servoing_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/visual_servoing_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/visual_servoing_test.dir/build: devel/lib/visual_servoing/visual_servoing_test

.PHONY : CMakeFiles/visual_servoing_test.dir/build

CMakeFiles/visual_servoing_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/visual_servoing_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/visual_servoing_test.dir/clean

CMakeFiles/visual_servoing_test.dir/depend:
	cd /home/joshimps/git/visualservoing/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/joshimps/git/visualservoing /home/joshimps/git/visualservoing /home/joshimps/git/visualservoing/build /home/joshimps/git/visualservoing/build /home/joshimps/git/visualservoing/build/CMakeFiles/visual_servoing_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/visual_servoing_test.dir/depend

