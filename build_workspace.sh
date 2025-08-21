#!/bin/bash

echo "🔨 Building Camera-LiDAR Fusion workspace..."

# Navigate to workspace
cd ~/CameraLidar/catkin_ws

# Source ROS
source /opt/ros/melodic/setup.bash

# Build workspace
echo "Building with catkin_make..."
catkin_make

# Source the workspace
source devel/setup.bash

echo "✅ Build complete!"
echo ""
echo "📋 Testing the build:"
echo "1. Check if package is found: rospack find camera_lidar_fusion"
echo "2. List launch files: ls src/camera_lidar_fusion/launch/"
echo "3. Run the system: roslaunch camera_lidar_fusion camera_lidar_fusion_simple.launch"
