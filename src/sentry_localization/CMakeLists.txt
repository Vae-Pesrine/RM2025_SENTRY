cmake_minimum_required(VERSION 3.0.2)
project(sentry_localization)

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
)

find_package(costmap_2d REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(Boost REQUIRED)

find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
if(OpenMP_FOUND)
  message(STATUS "OpenMP found: ${OpenMP_CXX_FLAGS}")
else()
  message(FATAL_ERROR "OpenMP not found!")
endif()

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES sentry_localization
  CATKIN_DEPENDS map_server move_base roscpp rospy std_msgs costmap_2d 
  DEPENDS 
)

include_directories(
  include
  /usr/local/include/teaser
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(initialpose_node src/initialpose_node.cpp)
add_dependencies(initialpose_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(initialpose_node 
  ${catkin_LIBRARIES}
)

add_executable(livox_lidar_to_points src/livox_repub/livox_lidar_to_points.cpp)
add_dependencies(livox_lidar_to_points ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(livox_lidar_to_points 
  ${catkin_LIBRARIES}
)

add_executable(small_gicp_relocalization src/small_gicp_relocalization.cpp)
add_dependencies(small_gicp_relocalization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(small_gicp_relocalization  
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
)

add_executable(test_pcl src/test.cpp)
add_dependencies(test_pcl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_pcl  
  ${catkin_LIBRARIES}
)


