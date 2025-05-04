cmake_minimum_required(VERSION 3.0.2)
project(scqn)

### set compiler
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17 -pthread -fexceptions -g -ggdb") #c++17, for parallel programming and for Eigen (see https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html)
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
include(FindOpenMP) #For Nano-GICP, Quatro #The best way to set proper compiler settings for using OpenMP in all platforms
if(OPENMP_FOUND) #For Nano-GICP, Quatro #The best way to set proper compiler settings for using OpenMP in all platforms
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else(OPENMP_FOUND)
  message("ERROR: OpenMP could not be found.")
endif(OPENMP_FOUND)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

# get packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  nano_gicp
  nav_msgs
  pcl_conversions
  pcl_ros
  quatro
  rosbag
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  visualization_msgs
)
find_package(PCL 1.12 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
if(PCL_FOUND)
  message(WARNING "PCL_VER: ${PCL_VERSION}, and PCL_DIR: ${PCL_INCLUDE_DIRS}")
endif()
if(Eigen3_FOUND)
  message(WARNING "EIGEN_VER: ${EIGEN3_VERSION_STRING}, and Eigen DIR: ${EIGEN3_INCLUDE_DIR}")
endif()
if(GTSAM_FOUND)
  message(WARNING "GTSAM_VER: ${GTSAM_VERSION}, and GTSAM DIR: ${GTSAM_INCLUDE_DIR}")
endif()

catkin_package()

### get packages' headers
include_directories(
  include
  ${CMAKE_SOURCE_DIR}/../../thirdparty/scancontext_tro
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
)

set(COMMON_LIBS 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBS} 
  ${OpenMP_LIBS} 
  gtsam
)
add_library(${PROJECT_NAME}_modules 
  ${CMAKE_SOURCE_DIR}/../../thirdparty/scancontext_tro/Scancontext.cpp 
  src/loop_closure.cpp
  src/map_matcher.cpp
)
target_link_libraries(${PROJECT_NAME}_modules ${COMMON_LIBS})

add_executable(scqn_sam_node src/scqn_sam.cpp)
target_link_libraries(scqn_sam_node ${COMMON_LIBS} ${PROJECT_NAME}_modules)

add_executable(scqn_localization_node src/scqn_localization.cpp)
target_link_libraries(scqn_localization_node ${COMMON_LIBS} ${PROJECT_NAME}_modules)
