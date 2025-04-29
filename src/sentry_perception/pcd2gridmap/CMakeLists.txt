cmake_minimum_required(VERSION 3.0.2)
project(pcd2gridmap)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  pcl_conversions

  roscpp
  sensor_msgs
  std_msgs
)
find_package(PCL 1.12 REQUIRED)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES pcd2gridmap
  CATKIN_DEPENDS roscpp std_msgs pcl_ros pcl_conversions sensor_msgs 
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pcd_to_gridmap src/pcd_to_gridmap.cpp)
add_dependencies(pcd_to_gridmap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcd_to_gridmap 
${catkin_LIBRARIES}
)

