cmake_minimum_required(VERSION 3.0.2)
project(gridmap_creator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs 
# )

# catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gridmap_creator
# CATKIN_DEPENDS roscpp 
#  DEPENDS system_lib
# )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
)
link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(msgs)

add_library(gridmap_creator SHARED src/gridmap_creator.cpp )
target_link_libraries(gridmap_creator collision_map_creator_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(gridmap_creator collision_map_creator_msgs)

add_executable (request_publisher src/request_publisher.cpp)
target_link_libraries(request_publisher pthread collision_map_creator_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(request_publisher collision_map_creator_msgs)
