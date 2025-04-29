cmake_minimum_required(VERSION 3.0.2)
project(terrain_analysis)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  pcl_conversions
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
)

find_package(PCL 1.12 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES terrain_analysis
 CATKIN_DEPENDS pcl_ros roscpp rospy std_msgs nav_msgs sensor_msgs
 DEPENDS PCL 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(terrain_analysis_node src/terrain_analysis.cpp)
add_dependencies(terrain_analysis_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(terrain_analysis_node  
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)



