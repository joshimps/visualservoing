# CMake generated Testfile for 
# Source directory: /home/joshimps/git/visualservoing
# Build directory: /home/joshimps/git/visualservoing/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_visual_servoing_gtest_robotControllerTests "/home/joshimps/git/visualservoing/build/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/joshimps/git/visualservoing/build/test_results/visual_servoing/gtest-robotControllerTests.xml" "--return-code" "/home/joshimps/git/visualservoing/build/devel/lib/visual_servoing/robotControllerTests --gtest_output=xml:/home/joshimps/git/visualservoing/build/test_results/visual_servoing/gtest-robotControllerTests.xml")
set_tests_properties(_ctest_visual_servoing_gtest_robotControllerTests PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/joshimps/git/visualservoing/CMakeLists.txt;147;catkin_add_gtest;/home/joshimps/git/visualservoing/CMakeLists.txt;0;")
add_test(_ctest_visual_servoing_gtest_robotTests "/home/joshimps/git/visualservoing/build/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/joshimps/git/visualservoing/build/test_results/visual_servoing/gtest-robotTests.xml" "--return-code" "/home/joshimps/git/visualservoing/build/devel/lib/visual_servoing/robotTests --gtest_output=xml:/home/joshimps/git/visualservoing/build/test_results/visual_servoing/gtest-robotTests.xml")
set_tests_properties(_ctest_visual_servoing_gtest_robotTests PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/joshimps/git/visualservoing/CMakeLists.txt;148;catkin_add_gtest;/home/joshimps/git/visualservoing/CMakeLists.txt;0;")
subdirs("gtest")
