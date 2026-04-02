cmake_minimum_required(VERSION 3.0.2)
project(cam_lidar_planner_ros)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  cv_bridge
  image_transport
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cam_lidar_planner_ros
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs cv_bridge image_transport
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(cam_lidar_planner_ros
  src/local_planner.cpp
  src/APF.cpp
  src/DWA.cpp
  src/ObstacleMemoryCam.cpp
  src/planner_controller.cpp
)


target_link_libraries(
  cam_lidar_planner_ros
  ${catkin_LIBRARIES}
)

