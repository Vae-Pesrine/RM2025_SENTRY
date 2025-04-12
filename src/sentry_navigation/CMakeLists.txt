cmake_minimum_required(VERSION 3.0.2)
project(sentry_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf2
  nav_msgs
  std_msgs
  sensor_msgs
  pcl_ros
  message_generation
)

find_package(PCL 1.12 REQUIRED)
find_package(Boost REQUIRED)

add_message_files(
  FILES
  refree.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
 CATKIN_DEPENDS 
  roscpp
  tf
  nav_msgs
  sensor_msgs
  pcl_ros
  std_msgs
  message_runtime
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
link_directories(${PCL_LIBRARY_DIRS})


add_executable(test_goal src/test_goal.cpp)
add_dependencies(test_goal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_goal
  ${catkin_LIBRARIES}
)

