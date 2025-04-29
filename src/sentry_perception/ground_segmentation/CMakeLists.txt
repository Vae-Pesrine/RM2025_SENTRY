cmake_minimum_required(VERSION 3.0.2)
project(ground_segmentation)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  std_msgs
  tf2_ros
  eigen_conversions
)

find_package(PCL 1.12 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread date_time)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ground_segmentation
 CATKIN_DEPENDS pcl_ros roscpp rospy std_msgs tf2_ros eigen_conversions
 DEPENDS PCL Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

file(GLOB SOURCES "src/*.cpp")
add_library(ground_segmentation ${SOURCES})
add_dependencies(ground_segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ground_segmentation  
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)


add_executable(ground_segmentation_node src/ground_segmentation_node.cpp)
add_dependencies(ground_segmentation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ground_segmentation_node  
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ground_segmentation
)



