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
find_package(OpenCV REQUIRED)

find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(OpenMP_LIBRARIES ${OpenMP_CXX_LIBRARIES})
  set(OpenMP_INCLUDE_DIRS ${OpenMP_CXX_INCLUDE_DIRS})
  message(STATUS "OpenMP found: ${OpenMP_CXX_FLAGS}")
else()
  message(FATAL_ERROR "OpenMP not found!")
endif()

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES small_gicp_relocalization
  CATKIN_DEPENDS roscpp  pcl_ros rospy 
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(small_gicp_relocalization_node src/small_gicp_relocalization.cpp)
add_dependencies(small_gicp_relocalization_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(small_gicp_relocalization_node  
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
)

add_library(global_localization_lib
  src/global_localization/bbs_localization.cpp
  src/global_localization/tools.cpp
  src/global_localization/global_localization.cpp
)

target_link_libraries(global_localization_lib
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(test_pcl src/test.cpp)
add_dependencies(test_pcl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_pcl  
  global_localization_lib  
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)