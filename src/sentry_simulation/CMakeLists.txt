cmake_minimum_required(VERSION 2.8.3)
project(sentry_simulation)
add_compile_options(-std=c++17)

set(CMAKE_BUILD_TYPE Release)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
  tf
  message_generation
  livox_ros_driver2
)

find_package(PCL REQUIRED)
find_package(gazebo REQUIRED)

#message
# add_message_files(FILES
#  CustomPoint.msg
#  CustomMsg.msg
# )

# generate_messages(DEPENDENCIES
# std_msgs
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs tf pcl_ros message_runtime
  INCLUDE_DIRS include
  LIBRARIES sentry_simulation
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
  ~/RMUC_simulation/src/sentry_simulation/include
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)

## Specify additional locations for library files
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)

add_executable(key_control ${PROJECT_SOURCE_DIR}/src/key_control.cpp)
target_link_libraries(key_control ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(sentry_simulation SHARED src/livox_points_plugin.cpp src/livox_ode_multiray_shape.cpp)
target_link_libraries(sentry_simulation ${catkin_LIBRARIES} ${PCL_LIBRARIES} RayPlugin)
target_link_libraries(sentry_simulation libprotobuf.so.9)
