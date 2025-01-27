cmake_minimum_required(VERSION 3.0.2)
project(sentry_localization)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  map_server
  move_base
  roscpp
  rospy
  std_msgs
  costmap_2d
  pcl_ros
  livox_ros_driver2
  filters
  message_filters
  nodelet
)

find_package(costmap_2d REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES sentry_localization
  CATKIN_DEPENDS map_server move_base roscpp rospy std_msgs costmap_2d 
  DEPENDS 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  # /usr/include/pcl-pcl-1.12.0/common/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})


add_executable(initialpose_relocation src/initialpose/initialpose_relocation.cpp)
add_dependencies(initialpose_relocation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(initialpose_relocation 
${catkin_LIBRARIES}
)

add_executable(initialpose_node src/initialpose/initialpose_node.cpp)
add_dependencies(initialpose_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(initialpose_node 
${catkin_LIBRARIES}
)

add_executable(livox_lidar_to_points src/livox_repub/livox_lidar_to_points.cpp)
add_dependencies(livox_lidar_to_points ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(livox_lidar_to_points 
${catkin_LIBRARIES}
)

add_executable(scan_to_scan_filter src/livox_repub/scan_to_scan_filter.cpp)
add_dependencies(scan_to_scan_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scan_to_scan_filter 
${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

add_library(pointcloud_to_laserscan src/livox_repub/pointcloud_to_laserscan_nodelet.cpp)
target_link_libraries(pointcloud_to_laserscan 
${catkin_LIBRARIES}
)

add_executable(pointcloud_to_laserscan_node src/livox_repub/pointcloud_to_laserscan.cpp)
add_dependencies(pointcloud_to_laserscan_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointcloud_to_laserscan_node pointcloud_to_laserscan 
${catkin_LIBRARIES}
)

add_executable(small_gicp_relocalization src/small_gicp/small_gicp_relocalization.cpp)
add_dependencies(small_gicp_relocalization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(small_gicp_relocalization pointcloud_to_laserscan 
${catkin_LIBRARIES}
)

