#!/bin/bash

echo "🚀 Setting up Camera-LiDAR Fusion on Jetson (Minimal Setup)..."

# Update system
echo "📦 Updating system packages..."
sudo apt update

# Install catkin-tools
echo "🔧 Installing catkin-tools..."
sudo apt install python3-catkin-tools -y

# Install minimal ROS dependencies
echo "📦 Installing minimal ROS dependencies..."
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
pip3 install numpy opencv-python

# Set up workspace
echo "🏗️  Setting up workspace..."
cd ~/CameraLidar/catkin_ws

# Try catkin build first, fallback to catkin_make
if command -v catkin &> /dev/null; then
    echo "🔨 Building with catkin build..."
    catkin build
else
    echo "🔨 Building with catkin_make..."
    catkin_make
fi

# Source the workspace
if [ -f "devel/setup.bash" ]; then
    source devel/setup.bash
    echo "✅ Workspace sourced successfully"
fi

echo "✅ Minimal setup complete!"
echo ""
echo "📋 Next steps:"
echo "1. Install ZED SDK for Tegra"
echo "2. Install UniLiDAR SDK"
echo "3. Run: roslaunch camera_lidar_fusion camera_lidar_fusion_simple.launch"
echo ""
echo "💡 This minimal setup uses the simplified fusion node that doesn't require PCL Python bindings."
