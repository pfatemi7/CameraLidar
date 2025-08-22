#!/bin/bash

# Camera-LiDAR Fusion System Startup Script
echo "=== Starting Camera-LiDAR Fusion System ==="

# Navigate to workspace
cd ~/CameraLidar/clean_ws

# Source ROS environment
source devel/setup.bash

# Set display for GUI applications
export DISPLAY=:0

echo "Starting complete system..."
echo "This will launch:"
echo "- ZED Camera"
echo "- UniLiDAR"
echo "- LiDAR Processing"
echo "- Camera-LiDAR Fusion"
echo "- Distance Monitor"
echo "- RViz Visualization"
echo "- Image Viewers"
echo "- Web Monitor"
echo ""
echo "Press Ctrl+C to stop all components"
echo ""

# Launch the complete system
roslaunch camera_lidar_fusion complete_system_final.launch
