#!/bin/bash

echo "🚀 Installing ROS Melodic on Jetson (Ubuntu 18.04)..."

# Update system
echo "📦 Updating system packages..."
sudo apt update && sudo apt upgrade -y

# Install ROS Melodic
echo "📦 Installing ROS Melodic..."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop-full -y

# Install build tools
echo "🔧 Installing build tools..."
sudo apt install python-catkin-tools python-catkin-pkg-modules python-rospkg-modules -y

# Install dependencies
echo "📦 Installing ROS dependencies..."
sudo apt install -y \
    python-pip \
    ros-melodic-cv-bridge \
    ros-melodic-image-transport \
    ros-melodic-message-filters \
    ros-melodic-tf2-ros \
    ros-melodic-rviz \
    ros-melodic-geometry-msgs \
    ros-melodic-sensor-msgs \
    ros-melodic-std-msgs \
    ros-melodic-tf

# Install Python dependencies
echo "🐍 Installing Python dependencies..."
pip install numpy
pip install opencv-python-headless  # Use headless version for Jetson

# Initialize ROS
echo "🔧 Initializing ROS..."
source /opt/ros/melodic/setup.bash
sudo rosdep init
rosdep update

echo "✅ ROS Melodic installation complete!"
echo ""
echo "📋 Next steps:"
echo "1. Source ROS: source /opt/ros/melodic/setup.bash"
echo "2. Build workspace: cd ~/CameraLidar/catkin_ws && catkin_make"
echo "3. Run: roslaunch camera_lidar_fusion camera_lidar_fusion_simple.launch"
