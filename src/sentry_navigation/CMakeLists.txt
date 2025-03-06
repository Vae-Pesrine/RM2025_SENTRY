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
  nav_msgs
  std_msgs
  sensor_msgs
  pcl_ros
)

find_package(PCL 1.12 REQUIRED)
find_package(Boost REQUIRED)


catkin_package(
 CATKIN_DEPENDS 
  roscpp
  tf
  nav_msgs
  sensor_msgs
  pcl_ros
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


add_executable(terrain_analysis src/terrian_analysis/terrainAnalysis.cpp)
add_dependencies(terrain_analysis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(terrain_analysis
  ${catkin_LIBRARIES}
)

add_executable(terrain_analysis_ext src/terrian_analysis/terrainAnalysisExt.cpp)
add_dependencies(terrain_analysis_ext ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(terrain_analysis_ext
  ${catkin_LIBRARIES}
)

add_executable(test_goal src/test_goal.cpp)
add_dependencies(test_goal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_goal
  ${catkin_LIBRARIES}
)