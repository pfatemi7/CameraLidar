cmake_minimum_required(VERSION 3.0.2)
project(camera_lidar_fusion)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  cv_bridge
  image_transport
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    pcl_ros
    pcl_conversions
    cv_bridge
    image_transport
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add executable for C++ node
add_executable(camera_lidar_fusion_node src/camera_lidar_fusion_node.cpp)
target_link_libraries(camera_lidar_fusion_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Add Python scripts
catkin_install_python(PROGRAMS
  scripts/camera_lidar_fusion_node.py
  scripts/calibration_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
