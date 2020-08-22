cmake_minimum_required(VERSION 3.0.2)
project(gps_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  serial
  roslaunch
  gps_common
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES digi_ros_com
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(gps_node
  include/gps_driver.h
  src/gps_node.cpp
  src/gps_driver.cpp
)
add_dependencies(gps_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_node ${catkin_LIBRARIES})
