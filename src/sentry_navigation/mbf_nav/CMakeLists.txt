cmake_minimum_required(VERSION 3.0.2)
project(mbf_nav)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  mbf_msgs
  roscpp
  rospy
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES mbf_nav
 CATKIN_DEPENDS mbf_msgs roscpp rospy geometry_msgs
 DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mbf_bridge src/mbf_bridge.cpp)
add_dependencies(mbf_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mbf_bridge
  ${catkin_LIBRARIES}
)


