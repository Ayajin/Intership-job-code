cmake_minimum_required(VERSION 3.0.2)
project(tile_id_encoder)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2
  std_msgs
  geometry_msgs
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy std_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)