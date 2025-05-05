cmake_minimum_required(VERSION 3.0.2)
project(relocalization)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  pcl_ros
)

find_package(PCL 1.12 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# OpenMP
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

# GPU BBS
find_path(gpu_bbs3d_INCLUDE_DIR gpu_bbs3d
  HINTS /usr/local/include)
find_library(gpu_bbs3d_LIBRARY NAMES gpu_bbs3d
  HINTS /usr/local/lib)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(gpu_bbs3d DEFAULT_MSG gpu_bbs3d_INCLUDE_DIR gpu_bbs3d_LIBRARY)

find_package(CUDA)
if(CUDA_FOUND)
  include_directories(${CUDA_INCLUDE_DIRS})
  link_directories(${CUDA_LIBRARY_DIRS})
endif()

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES relocalization
  CATKIN_DEPENDS roscpp  pcl_ros rospy 
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${gpu_bbs3d_INCLUDE_DIR}
)

add_executable(relocalization_node src/relocalization.cpp)
add_dependencies(relocalization_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(relocalization_node  
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  ${gpu_bbs3d_LIBRARY}
  ${CUDA_LIBRARIES}
)