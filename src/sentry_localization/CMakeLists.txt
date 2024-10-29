cmake_minimum_required(VERSION 3.0.2)
project(sentry_localization)
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(catkin REQUIRED COMPONENTS
  nodelet  
  pcl_ros
  pcl_conversions
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  eigen_conversions
  fast_gicp
  ndt_omp
  sentry_userdefinition
)
find_package(PCL 1.7 REQUIRED COMPOMENTS common io)
include_directories(${PCL_INCLUDE_DIRS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(OpenCV REQUIRED)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp std_msgs nodelet
)

add_library(globalmap_server_nodelet 
  nodelet/globalmap_server_nodelet.cpp
)
target_link_libraries(globalmap_server_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(sentry_localization_nodelet 
  nodelet/sentry_localization_nodelet.cpp
  src/pose_estimator.cpp
)
target_link_libraries(sentry_localization_nodelet 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(global_localization
  apps/bbs_localization.cpp
  apps/global_localization_bbs.cpp
  apps/global_localization_fpfh_ransac.cpp
  apps/ransac_pose_estimation.cpp
  apps/voxelset.cpp
)
target_link_libraries(global_localization
  ${OpenCV_LIBRARIES}
)

add_executable(global_localization_node src/global_localization.cpp)
add_dependencies(global_localization_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(global_localization_node
  global_localization
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


add_executable(livox_lidar_to_points src/livox_repub.cpp)
add_dependencies(livox_lidar_to_points ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(livox_lidar_to_points
  ${catkin_LIBRARIES}
)
