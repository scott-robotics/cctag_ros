cmake_minimum_required(VERSION 3.0.2)
# cmake_policy(SET CMP0048 NEW)
project(cctag_ros)



## Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost REQUIRED COMPONENTS serialization)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cctag_ros
  CATKIN_DEPENDS roscpp
)


###########
## Build ##
###########
add_definitions(-DCCTAG_WITH_CUDA)
add_subdirectory(CCTag)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# BOOST
set(BOOST_REQUIRED_COMPONENTS "date_time;chrono;thread;serialization;system;filesystem;atomic;program_options;timer")
set(BOOST_REQUIRED_COMPONENTS "${BOOST_REQUIRED_COMPONENTS};stacktrace_basic")

find_package(Boost 1.66.0 REQUIRED COMPONENTS ${BOOST_REQUIRED_COMPONENTS} QUIET)

# OPENCV
# find_package(OpenCV REQUIRED core videoio imgproc imgcodecs highgui)
find_package(OpenCV REQUIRED)

# TBB
find_package(TBB REQUIRED)

set(CCTagDetect_cpp ./nodes/cctag_ros.cpp ./CCTag/src/applications/detection/CmdLine.cpp)
add_executable(cctag_ros ${CCTagDetect_cpp})

target_include_directories(cctag_ros PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ./CCTag/src/applications/detection)

target_link_libraries(cctag_ros
  CCTag::CCTag
  TBB::tbb
  ${OpenCV_LIBRARIES}
  Boost::filesystem Boost::program_options Boost::timer
  ${catkin_LIBRARIES}
  )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
