cmake_minimum_required(VERSION 3.0.2)
project(move_freespace_recovery)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_core
  pluginlib
  costmap_2d
  geometry_msgs
  move_base_msgs
  visualization_msgs
  tf2
  tf2_ros
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES move_freespace_recovery
 CATKIN_DEPENDS roscpp rospy nav_core pluginlib move_base_msgs geometry_msgs visualization_msgs
 DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(move_towards_free_space src/test_recovery.cpp)
add_dependencies(move_towards_free_space ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_towards_free_space
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

