#!/bin/bash

echo "🚀 Setting up Camera-LiDAR Fusion on Jetson..."

# Update system
echo "📦 Updating system packages..."
sudo apt update && sudo apt upgrade -y

# Install ROS Noetic (if not already installed)
if ! command -v roscore &> /dev/null; then
    echo "📦 Installing ROS Noetic..."
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    sudo apt update
    sudo apt install ros-noetic-desktop-full -y
fi

# Install catkin-tools
echo "🔧 Installing catkin-tools..."
sudo apt install python3-catkin-tools -y

# Install ROS dependencies one by one to handle errors
echo "📦 Installing ROS dependencies..."
packages=(
    "python3-pip"
    "ros-noetic-pcl-ros"
    "ros-noetic-pcl-conversions"
    "ros-noetic-cv-bridge"
    "ros-noetic-image-transport"
    "ros-noetic-message-filters"
    "ros-noetic-tf2-ros"
    "ros-noetic-rviz"
    "ros-noetic-rqt-plot"
    "ros-noetic-geometry-msgs"
    "ros-noetic-sensor-msgs"
    "ros-noetic-std-msgs"
    "ros-noetic-tf"
    "ros-noetic-tf2-geometry-msgs"
)

for package in "${packages[@]}"; do
    echo "Installing $package..."
    if sudo apt install -y "$package"; then
        echo "✅ $package installed successfully"
    else
        echo "⚠️  Failed to install $package, continuing..."
    fi
done

# Install Python dependencies
echo "🐍 Installing Python dependencies..."
pip3 install numpy opencv-python

# Try to install PCL for Python (may not work on all systems)
echo "📊 Installing PCL Python bindings..."
if pip3 install python-pcl; then
    echo "✅ PCL Python bindings installed"
else
    echo "⚠️  PCL Python bindings not available, will use simplified version"
fi

# Set up workspace
echo "🏗️  Setting up workspace..."
cd ~/CameraLidar/catkin_ws

# Try catkin build first, fallback to catkin_make
if command -v catkin &> /dev/null; then
    echo "🔨 Building with catkin build..."
    if catkin build; then
        echo "✅ Build successful with catkin build"
    else
        echo "⚠️  catkin build failed, trying catkin_make..."
        catkin_make
    fi
else
    echo "🔨 Building with catkin_make..."
    catkin_make
fi

# Source the workspace
if [ -f "devel/setup.bash" ]; then
    source devel/setup.bash
    echo "✅ Workspace sourced successfully"
else
    echo "⚠️  Could not source workspace, build may have failed"
fi

# Set permissions for LiDAR (adjust device name as needed)
echo "🔐 Setting LiDAR permissions..."
if [ -e "/dev/ttyUSB0" ]; then
    sudo chmod 666 /dev/ttyUSB0
    echo "✅ LiDAR permissions set"
else
    echo "⚠️  LiDAR device not found, set permissions manually when connected"
fi

echo "✅ Setup complete!"
echo ""
echo "📋 Next steps:"
echo "1. Install ZED SDK for Tegra from: https://www.stereolabs.com/docs/installation/linux/"
echo "2. Install UniLiDAR SDK"
echo "3. Run: roslaunch camera_lidar_fusion calibration.launch"
echo "4. Run: roslaunch camera_lidar_fusion camera_lidar_fusion_simple.launch"
echo ""
echo "🔧 If you encounter issues:"
echo "- Check ROS installation: roscore"
echo "- Check workspace: source devel/setup.bash"
echo "- Use simple version if PCL bindings fail: camera_lidar_fusion_simple.launch"
