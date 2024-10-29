cmake_minimum_required(VERSION 3.0.2)
project(sentry_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
  tf2
  tf2_ros
  tf2_sensor_msgs
  filters
  message_filters
  nodelet
)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED COMPONENT common io visualization filters)



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pointcloud_to_laserscan
  CATKIN_DEPENDS pcl_ros roscpp rospy std_msgs message_filters nodelet sensor_msgs tf2 tf2_ros tf2_sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(scan_to_scan_filter src/scan_to_scan_filter.cpp)
target_link_libraries(scan_to_scan_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(pointcloud_to_laserscan src/pointcloud_to_laserscan_nodelet.cpp)
target_link_libraries(pointcloud_to_laserscan ${catkin_LIBRARIES})

add_executable(pointcloud_to_laserscan_node src/pointcloud_to_laserscan.cpp)
target_link_libraries(pointcloud_to_laserscan_node pointcloud_to_laserscan ${catkin_LIBRARIES})


