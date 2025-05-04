cmake_minimum_required(VERSION 3.0.2)
project(livox_msgs_convert)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  sensor_msgs
  livox_ros_driver2
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES livox_msgs_convert
 CATKIN_DEPENDS rospy roscpp sensor_msgs
 DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(  
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(livox_to_pointcloud2_node src/livox_to_pointcloud2.cpp)
add_dependencies(livox_to_pointcloud2_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(livox_to_pointcloud2_node
  ${catkin_LIBRARIES}
)