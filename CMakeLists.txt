cmake_minimum_required(VERSION 3.0.2)
project(visual_servoing)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf2
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include 
  src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GTEST_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_robot src/robot.cpp )
target_link_libraries(${PROJECT_NAME}_robot  ${catkin_LIBRARIES})

## Declare a C++ library
add_library(${PROJECT_NAME}_robotController src/robotController.cpp) 
target_link_libraries(${PROJECT_NAME}_robotController ${PROJECT_NAME}_robot ${catkin_LIBRARIES})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME} src/main.cpp src/robotController.cpp src/robot.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PROJECT_NAME}_robot ${PROJECT_NAME}_robotController)

#############
## Testing ##
#############

catkin_add_gtest(robotControllerTests test/robotControllerTests.cpp)
catkin_add_gtest(robotTests test/robotTests.cpp)

target_link_libraries(robotControllerTests
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_robotController
  ${PROJECT_NAME}_robot
  pthread
)

target_link_libraries(robotTests
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_robot
  pthread
)

