cmake_minimum_required(VERSION 3.0.2)
project(sentry_navigation)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  move_base_msgs
  mbf_msgs

  nav_core
  pluginlib
  costmap_2d
  tf2
  tf2_ros
  geometry_msgs
  visualization_msgs
)

find_package(Boost REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS 
    roscpp 
    actionlib 
    move_base_msgs
    nav_core
    pluginlib
    geometry_msgs
    visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_goal src/test_goal.cpp)
add_dependencies(test_goal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_goal
  ${catkin_LIBRARIES}
)

add_executable(mbf_bridge src/mbf_bridge.cpp)
add_dependencies(mbf_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mbf_bridge
  ${catkin_LIBRARIES}
)


add_library(move_towards_free_space src/test_recovery.cpp)
add_dependencies(move_towards_free_space ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_towards_free_space
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
