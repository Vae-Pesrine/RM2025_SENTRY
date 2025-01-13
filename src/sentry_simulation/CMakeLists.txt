cmake_minimum_required(VERSION 2.8.3)
project(sentry_simulation)

## Compile as C++11, supported in ROS Kinetic and newer
# updated to c++17, supported ubuntu 20.04 and ros noetic
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  pcl_ros
)

find_package(PCL REQUIRED)
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})

#To solve the error which gazebo does not handle the include file well, we must add this line.
include_directories(/usr/include/gazebo-11/gazebo)

link_directories(${GAZEBO_LIBRARY_DIRS})

#This is designed for whose proto installed in system is not 2.6. We can install the version of proto in local dir
#include_directories(/home/lfc/proto/include/)
#link_directories(/home/lfc/proto/lib/)

include_directories(
        include
        ${PCL_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES sentry_simulation
        CATKIN_DEPENDS tf
        pcl_ros
)

add_library(sentry_simulation SHARED src/livox_mid360/livox_points_plugin.cpp src/livox_mid360/livox_ode_multiray_shape.cpp)
target_link_libraries(sentry_simulation ${catkin_LIBRARIES} ${PCL_LIBRARIES} RayPlugin)
target_link_libraries(sentry_simulation libprotobuf.so.9)

add_executable(telelop_keyboard_ctrl src/telelop_keyboard_ctrl.cpp)
add_dependencies(telelop_keyboard_ctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(telelop_keyboard_ctrl 
${catkin_LIBRARIES}
)
