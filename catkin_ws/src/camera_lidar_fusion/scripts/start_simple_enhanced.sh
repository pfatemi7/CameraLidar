#!/bin/bash

# Simple Enhanced Camera-LiDAR Fusion System Startup Script for Jetson Nano
# Uses existing components with optimized parameters

echo "=========================================="
echo "Simple Enhanced Camera-LiDAR Fusion System"
echo "Jetson Nano Optimized Version"
echo "=========================================="

# Set display for Jetson Nano
export DISPLAY=:0

# Performance optimizations for Jetson Nano
echo "Applying Jetson Nano performance optimizations..."

# Set CPU governor to performance mode (if possible)
if [ -f /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor ]; then
    echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor 2>/dev/null || true
fi

# Check available memory
FREE_MEM=$(free -m | grep Mem | awk '{print $7}')
echo "Available memory: ${FREE_MEM}MB"

if [ $FREE_MEM -lt 1000 ]; then
    echo "Warning: Low memory available. Consider closing other applications."
fi

# Set environment variables for better performance
export CUDA_VISIBLE_DEVICES=0
export OMP_NUM_THREADS=4  # Limit OpenMP threads for Nano
export OPENBLAS_NUM_THREADS=4
export MKL_NUM_THREADS=4

# Set ROS parameters for Jetson Nano
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

# Check if ROS core is running
if ! pgrep -x "roscore" > /dev/null; then
    echo "Starting ROS core..."
    roscore &
    sleep 3
else
    echo "ROS core already running"
fi

# Check hardware connections
echo "Checking hardware connections..."

# Check ZED camera
if [ -e "/dev/video0" ]; then
    echo "✓ ZED camera detected"
else
    echo "✗ ZED camera not found. Please check USB connection."
    exit 1
fi

# Check LiDAR
if [ -e "/dev/ttyUSB0" ]; then
    echo "✓ LiDAR detected on /dev/ttyUSB0"
elif [ -e "/dev/ttyUSB1" ]; then
    echo "✓ LiDAR detected on /dev/ttyUSB1"
    # Update launch file to use correct port
    sed -i 's/\/dev\/ttyUSB0/\/dev\/ttyUSB1/g' $(find . -name "simple_enhanced_jetson.launch")
else
    echo "✗ LiDAR not found. Please check USB connection."
    exit 1
fi

# Check if we're in the right directory
if [ ! -f "package.xml" ]; then
    echo "Changing to catkin workspace..."
    cd ~/CameraLidar/catkin_ws
fi

# Source ROS environment
echo "Sourcing ROS environment..."
source devel/setup.bash

# Make scripts executable
echo "Making scripts executable..."
chmod +x src/camera_lidar_fusion/scripts/*.py

# Launch the simple enhanced system
echo "Launching Simple Enhanced Camera-LiDAR Fusion System..."
echo "This may take a few moments to initialize..."

# Launch with reduced priority to prevent system overload
nice -n 10 roslaunch camera_lidar_fusion simple_enhanced_jetson.launch

# If launch fails, try with minimal configuration
if [ $? -ne 0 ]; then
    echo "Simple enhanced system launch failed. Trying original system..."
    echo "This will run with basic features for compatibility."
    
    # Launch original system
    nice -n 10 roslaunch camera_lidar_fusion complete_system_final.launch
fi

echo "System shutdown complete."
