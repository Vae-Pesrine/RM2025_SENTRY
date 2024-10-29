cmake_minimum_required(VERSION 3.0.2)
project(move_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  map_server
  move_base
  roscpp
  rospy
  std_msgs
  costmap_2d
  dynamic_reconfigure
)

find_package(
  costmap_2d REQUIRED
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES move_controller
  CATKIN_DEPENDS map_server move_base roscpp rospy std_msgs costmap_2d dynamic_reconfigure geometry_msgs
  DEPENDS 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(
  /home/tjurm/mid_360_ws/src/move_controller/include
)

# Declare a C++ library


add_executable(relocation src/relocation.cpp)
target_link_libraries(relocation ${catkin_LIBRARIES})

add_executable(InitialPose src/InitialPose.cpp)
target_link_libraries(InitialPose ${catkin_LIBRARIES})
