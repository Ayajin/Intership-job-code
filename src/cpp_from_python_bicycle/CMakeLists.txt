cmake_minimum_required(VERSION 3.0.2)
project(cpp_from_python_bicycle)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf2_ros
  tf2
)
# instead of
# find_package(catkin REQUIRED)
# find_package(roscpp REQUIRED)
# find_package(std_msgs REQUIRED)
# ... etc ...

add_service_files(
  FILES
  SetModelInput.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  roscpp
  std_msgs
  geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Build get_arrow
add_executable(get_arrow src/get_arrow.cpp)
target_link_libraries(get_arrow ${catkin_LIBRARIES} ncurses)

## Build control_node
add_executable(control_node src/control_node.cpp)
target_link_libraries(control_node ${catkin_LIBRARIES})

## Build bicycle_model
add_executable(bicycle_model src/bicycle_model.cpp)
target_link_libraries(bicycle_model ${catkin_LIBRARIES})

## Build bicycle_marker
add_executable(bicycle_marker src/bicycle_marker.cpp)
target_link_libraries(bicycle_marker ${catkin_LIBRARIES})