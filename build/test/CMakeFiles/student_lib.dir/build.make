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
include test/CMakeFiles/student_lib.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/student_lib.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/student_lib.dir/flags.make

test/CMakeFiles/student_lib.dir/__/src/main.cpp.o: test/CMakeFiles/student_lib.dir/flags.make
test/CMakeFiles/student_lib.dir/__/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/joshimps/git/visualservoing/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/student_lib.dir/__/src/main.cpp.o"
	cd /home/joshimps/git/visualservoing/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/student_lib.dir/__/src/main.cpp.o -c /home/joshimps/git/visualservoing/src/main.cpp

test/CMakeFiles/student_lib.dir/__/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/student_lib.dir/__/src/main.cpp.i"
	cd /home/joshimps/git/visualservoing/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/joshimps/git/visualservoing/src/main.cpp > CMakeFiles/student_lib.dir/__/src/main.cpp.i

test/CMakeFiles/student_lib.dir/__/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/student_lib.dir/__/src/main.cpp.s"
	cd /home/joshimps/git/visualservoing/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/joshimps/git/visualservoing/src/main.cpp -o CMakeFiles/student_lib.dir/__/src/main.cpp.s

test/CMakeFiles/student_lib.dir/__/src/robot.cpp.o: test/CMakeFiles/student_lib.dir/flags.make
test/CMakeFiles/student_lib.dir/__/src/robot.cpp.o: ../src/robot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/joshimps/git/visualservoing/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object test/CMakeFiles/student_lib.dir/__/src/robot.cpp.o"
	cd /home/joshimps/git/visualservoing/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/student_lib.dir/__/src/robot.cpp.o -c /home/joshimps/git/visualservoing/src/robot.cpp

test/CMakeFiles/student_lib.dir/__/src/robot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/student_lib.dir/__/src/robot.cpp.i"
	cd /home/joshimps/git/visualservoing/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/joshimps/git/visualservoing/src/robot.cpp > CMakeFiles/student_lib.dir/__/src/robot.cpp.i

test/CMakeFiles/student_lib.dir/__/src/robot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/student_lib.dir/__/src/robot.cpp.s"
	cd /home/joshimps/git/visualservoing/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/joshimps/git/visualservoing/src/robot.cpp -o CMakeFiles/student_lib.dir/__/src/robot.cpp.s

test/CMakeFiles/student_lib.dir/__/src/robotController.cpp.o: test/CMakeFiles/student_lib.dir/flags.make
test/CMakeFiles/student_lib.dir/__/src/robotController.cpp.o: ../src/robotController.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/joshimps/git/visualservoing/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object test/CMakeFiles/student_lib.dir/__/src/robotController.cpp.o"
	cd /home/joshimps/git/visualservoing/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/student_lib.dir/__/src/robotController.cpp.o -c /home/joshimps/git/visualservoing/src/robotController.cpp

test/CMakeFiles/student_lib.dir/__/src/robotController.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/student_lib.dir/__/src/robotController.cpp.i"
	cd /home/joshimps/git/visualservoing/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/joshimps/git/visualservoing/src/robotController.cpp > CMakeFiles/student_lib.dir/__/src/robotController.cpp.i

test/CMakeFiles/student_lib.dir/__/src/robotController.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/student_lib.dir/__/src/robotController.cpp.s"
	cd /home/joshimps/git/visualservoing/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/joshimps/git/visualservoing/src/robotController.cpp -o CMakeFiles/student_lib.dir/__/src/robotController.cpp.s

# Object files for target student_lib
student_lib_OBJECTS = \
"CMakeFiles/student_lib.dir/__/src/main.cpp.o" \
"CMakeFiles/student_lib.dir/__/src/robot.cpp.o" \
"CMakeFiles/student_lib.dir/__/src/robotController.cpp.o"

# External object files for target student_lib
student_lib_EXTERNAL_OBJECTS =

devel/lib/libstudent_lib.so: test/CMakeFiles/student_lib.dir/__/src/main.cpp.o
devel/lib/libstudent_lib.so: test/CMakeFiles/student_lib.dir/__/src/robot.cpp.o
devel/lib/libstudent_lib.so: test/CMakeFiles/student_lib.dir/__/src/robotController.cpp.o
devel/lib/libstudent_lib.so: test/CMakeFiles/student_lib.dir/build.make
devel/lib/libstudent_lib.so: test/CMakeFiles/student_lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/joshimps/git/visualservoing/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library ../devel/lib/libstudent_lib.so"
	cd /home/joshimps/git/visualservoing/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/student_lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/student_lib.dir/build: devel/lib/libstudent_lib.so

.PHONY : test/CMakeFiles/student_lib.dir/build

test/CMakeFiles/student_lib.dir/clean:
	cd /home/joshimps/git/visualservoing/build/test && $(CMAKE_COMMAND) -P CMakeFiles/student_lib.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/student_lib.dir/clean

test/CMakeFiles/student_lib.dir/depend:
	cd /home/joshimps/git/visualservoing/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/joshimps/git/visualservoing /home/joshimps/git/visualservoing/test /home/joshimps/git/visualservoing/build /home/joshimps/git/visualservoing/build/test /home/joshimps/git/visualservoing/build/test/CMakeFiles/student_lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/student_lib.dir/depend

