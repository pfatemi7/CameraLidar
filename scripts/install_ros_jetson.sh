#!/bin/bash

echo "🚀 Installing ROS Noetic on Jetson..."

# Update system
echo "📦 Updating system packages..."
sudo apt update && sudo apt upgrade -y

# Install ROS Noetic
echo "📦 Installing ROS Noetic..."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full -y

# Install build tools
echo "🔧 Installing build tools..."
sudo apt install python3-catkin-tools python3-catkin-pkg-modules python3-rospkg-modules -y

# Install dependencies
echo "📦 Installing ROS dependencies..."
sudo apt install -y \
    python3-pip \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-message-filters \
    ros-noetic-tf2-ros \
    ros-noetic-rviz \
    ros-noetic-geometry-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    ros-noetic-tf

# Install Python dependencies
echo "🐍 Installing Python dependencies..."
pip3 install numpy
pip3 install opencv-python-headless  # Use headless version for Jetson

# Initialize ROS
echo "🔧 Initializing ROS..."
source /opt/ros/noetic/setup.bash
sudo rosdep init
rosdep update

echo "✅ ROS installation complete!"
echo ""
echo "📋 Next steps:"
echo "1. Source ROS: source /opt/ros/noetic/setup.bash"
echo "2. Build workspace: cd ~/CameraLidar/catkin_ws && catkin_make"
echo "3. Run: roslaunch camera_lidar_fusion camera_lidar_fusion_simple.launch"
