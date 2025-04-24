cmake_minimum_required(VERSION 3.0.2)
project(sentry_simulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  /usr/include/gazebo-11/gazebo
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES sentry_simulation
  CATKIN_DEPENDS tf
)

add_executable(key_control src/key_control.cpp)
add_dependencies(key_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(key_control 
${catkin_LIBRARIES}
)