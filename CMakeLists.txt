cmake_minimum_required(VERSION 3.0.2)
project(controllors_pkg)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES controllors_pkg
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(controllors_pkg_node src/scout_ctrl.cpp)
add_executable(controllors_pkg_node_2 src/scout_ctrl_2_kinemic.cpp)
add_executable(controllors_pkg_node_3 src/scout_ctrl_2_kinemic_for_cpath.cpp)
target_link_libraries(controllors_pkg_node ${catkin_LIBRARIES})
target_link_libraries(controllors_pkg_node_2 ${catkin_LIBRARIES})
target_link_libraries(controllors_pkg_node_3 ${catkin_LIBRARIES})