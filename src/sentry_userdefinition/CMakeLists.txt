cmake_minimum_required(VERSION 3.0.2)
project(sentry_userdefinition)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  ScanMatchingStatus.msg
  LocalizationInfo.msg
)

add_service_files(
  FILES
  SetGlobalMap.srv
  QueryGlobalLocalization.srv
  SetGlobalLocalizationEngine.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs std_msgs sensor_msgs
)

catkin_package(
 CATKIN_DEPENDS geometry_msgs std_msgs sensor_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
