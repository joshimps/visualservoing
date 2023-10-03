# CMake generated Testfile for 
# Source directory: /home/joshimps/git/visualservoing
# Build directory: /home/joshimps/git/visualservoing/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_visual_servoing_gtest_visual_servoing_test "/home/joshimps/git/visualservoing/build/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/joshimps/git/visualservoing/build/test_results/visual_servoing/gtest-visual_servoing_test.xml" "--return-code" "/home/joshimps/git/visualservoing/build/devel/lib/visual_servoing/visual_servoing_test --gtest_output=xml:/home/joshimps/git/visualservoing/build/test_results/visual_servoing/gtest-visual_servoing_test.xml")
set_tests_properties(_ctest_visual_servoing_gtest_visual_servoing_test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/joshimps/git/visualservoing/CMakeLists.txt;209;catkin_add_gtest;/home/joshimps/git/visualservoing/CMakeLists.txt;0;")
subdirs("gtest")
