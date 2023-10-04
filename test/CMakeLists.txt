#######################################################################
# Now we enable gtests - this section does not need to change
set(GTEST_ROOT "/usr/src/gtest" CACHE PATH "Path to googletest")
find_package(GTest REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})

if(NOT GTEST_LIBRARY)
   message("GTest library not found")
endif()

enable_testing()
###################################

# Just in case someone had the include files in seperate directory
include_directories(../include)
include_directories(..)

# This uses the complete student's library
aux_source_directory("../src" SRC_LIST)
list(REMOVE_ITEM SRC_LIST "/main.cpp")
add_library(student_lib ${SRC_LIST})

add_executable(robotControllerTests robotControllerTests.cpp)
target_link_libraries(robotControllerTests ${GTEST_LIBRARIES} linkcommand student_lib )

add_executable(robotTests robotTests.cpp)
target_link_libraries(robotTests ${GTEST_LIBRARIES} linkcommand student_lib )