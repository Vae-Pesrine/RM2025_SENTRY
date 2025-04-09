cmake_minimum_required(VERSION 3.0.2)
project(linefit_ground_segmentation_ros)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    pcl_ros
    roscpp
    rospy
    std_msgs
    ground_segmentation
)

find_package(PCL 1.12 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread date_time)

catkin_package(
  INCLUDE_DIRS  /home
  LIBRARIES 
  CATKIN_DEPENDS  roscpp rospy std_msgs 
  DEPENDS 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${ground_segmentation_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(ground_segmentation_node src/ground_segmentation_node.cc)
add_dependencies(ground_segmentation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(ground_segmentation_node 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)



