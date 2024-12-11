cmake_minimum_required(VERSION 3.0.2)
project(object_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_conversions
  pcl_ros
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Obstacle.msg
  ObstacleList.msg
  Road_extract.msg
  trackbox.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  sensor_msgs
)


## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
 CATKIN_DEPENDS roscpp message_generation std_msgs message_runtime sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/groundremove
  ${catkin_INCLUDE_DIRS}
  ${OpenCv_LIBRARIES}
  ${PCL_INCLUDE_DIRS}
)

set(SOURCE_FILES1 src/groundremove/main.cpp src/groundremove/gaus_blur.cpp src/groundremove/ground_removal.cpp)
add_executable(ground ${SOURCE_FILES1})
target_link_libraries(ground ${catkin_LIBRARIES} ${OpenCv_LIBRARIES})

# set(SOURCE_FILES2 src/cluster/main.cpp src/cluster/component_clustering.cpp src/cluster/box_fitting.cpp)
# add_executable(cluster ${SOURCE_FILES1})
# target_link_libraries(cluster ${catkin_LIBRARIES} ${OpenCv_LIBRARIES})

# set(SOURCE_FILES1 src/groundmove/main.cpp src/groundmove/gaus_blur.cpp src/groundmove/ground_removal.cpp)
# add_executable(ground ${SOURCE_FILES1})
# target_link_libraries(ground ${catkin_LIBRARIES} ${OpenCv_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_object_tracking.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
