cmake_minimum_required(VERSION 3.0.2)
project(digi_rf)

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

add_library(rf_driver
  src/rf_driver.cpp
)

target_link_libraries(rf_driver
  ${catkin_LIBRARIES}
)


add_executable(rf_node src/rf_node.cpp)
target_link_libraries(rf_node rf_driver ${catkin_LIBRARIES})


add_executable(rf_tx_beat src/rf_tx_beat.cpp)
target_link_libraries(rf_tx_beat ${catkin_LIBRARIES})
