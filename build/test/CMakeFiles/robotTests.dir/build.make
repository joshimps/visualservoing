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
include test/CMakeFiles/robotTests.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/robotTests.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/robotTests.dir/flags.make

test/CMakeFiles/robotTests.dir/robotTests.cpp.o: test/CMakeFiles/robotTests.dir/flags.make
test/CMakeFiles/robotTests.dir/robotTests.cpp.o: ../test/robotTests.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/joshimps/git/visualservoing/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/robotTests.dir/robotTests.cpp.o"
	cd /home/joshimps/git/visualservoing/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/robotTests.dir/robotTests.cpp.o -c /home/joshimps/git/visualservoing/test/robotTests.cpp

test/CMakeFiles/robotTests.dir/robotTests.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robotTests.dir/robotTests.cpp.i"
	cd /home/joshimps/git/visualservoing/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/joshimps/git/visualservoing/test/robotTests.cpp > CMakeFiles/robotTests.dir/robotTests.cpp.i

test/CMakeFiles/robotTests.dir/robotTests.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robotTests.dir/robotTests.cpp.s"
	cd /home/joshimps/git/visualservoing/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/joshimps/git/visualservoing/test/robotTests.cpp -o CMakeFiles/robotTests.dir/robotTests.cpp.s

# Object files for target robotTests
robotTests_OBJECTS = \
"CMakeFiles/robotTests.dir/robotTests.cpp.o"

# External object files for target robotTests
robotTests_EXTERNAL_OBJECTS =

test/robotTests: test/CMakeFiles/robotTests.dir/robotTests.cpp.o
test/robotTests: test/CMakeFiles/robotTests.dir/build.make
test/robotTests: /usr/lib/x86_64-linux-gnu/libgtest.a
test/robotTests: test/libstudent_lib.so
test/robotTests: test/CMakeFiles/robotTests.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/joshimps/git/visualservoing/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable robotTests"
	cd /home/joshimps/git/visualservoing/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/robotTests.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/robotTests.dir/build: test/robotTests

.PHONY : test/CMakeFiles/robotTests.dir/build

test/CMakeFiles/robotTests.dir/clean:
	cd /home/joshimps/git/visualservoing/build/test && $(CMAKE_COMMAND) -P CMakeFiles/robotTests.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/robotTests.dir/clean

test/CMakeFiles/robotTests.dir/depend:
	cd /home/joshimps/git/visualservoing/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/joshimps/git/visualservoing /home/joshimps/git/visualservoing/test /home/joshimps/git/visualservoing/build /home/joshimps/git/visualservoing/build/test /home/joshimps/git/visualservoing/build/test/CMakeFiles/robotTests.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/robotTests.dir/depend
