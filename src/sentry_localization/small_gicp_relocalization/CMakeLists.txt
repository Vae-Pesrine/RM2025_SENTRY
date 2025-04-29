cmake_minimum_required(VERSION 3.0.2)
project(small_gicp_relocalization)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_ros
)

find_package(PCL 1.12 REQUIRED)
find_package(Boost REQUIRED)

find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
if(OpenMP_FOUND)
  message(STATUS "OpenMP found: ${OpenMP_CXX_FLAGS}")
else()
  message(FATAL_ERROR "OpenMP not found!")
endif()

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES small_gicp_relocalization
 CATKIN_DEPENDS roscpp rospy pcl_ros
 DEPENDS PCL Boost OpenMP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(small_gicp_relocalization src/small_gicp_relocalization.cpp)
add_dependencies(small_gicp_relocalization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(small_gicp_relocalization  
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
)