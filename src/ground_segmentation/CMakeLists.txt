cmake_minimum_required(VERSION 3.0.2)
project(ground_segmentation)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    pcl_ros
    roscpp
    rospy
    std_msgs
)

find_package(PCL 1.12 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread date_time)



catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ground_segmentation
  CATKIN_DEPENDS  roscpp rospy std_msgs 
  DEPENDS PCL Boost
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
)

# 添加软链接目录到链接路径
link_directories(${PCL_LIBRARY_DIRS})

file(GLOB SOURCES "src/*.cc")
add_library(ground_segmentation ${SOURCES})
add_dependencies(ground_segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ground_segmentation  
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)