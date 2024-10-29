cmake_minimum_required(VERSION 3.0.2)
project(localization)

set(CMAKE_BUILD_TYPE Debug)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  livox_ros_driver2
  nav_msgs  
  sensor_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  fast_gicp
  ndt_omp
  sentry_userdefinition
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES localization
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(localization src/localization.cpp)
add_dependencies(localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(localization
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


