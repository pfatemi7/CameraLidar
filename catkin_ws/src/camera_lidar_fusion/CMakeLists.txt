cmake_minimum_required(VERSION 3.0.2)
project(camera_lidar_fusion)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_filters
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    message_filters
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Add Python scripts
catkin_install_python(PROGRAMS
  scripts/camera_lidar_fusion_simple.py
  scripts/calibration_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
