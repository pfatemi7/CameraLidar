#!/bin/bash

# Enhanced Camera-LiDAR Fusion System Startup Script for Jetson Nano
# Optimized for 4GB RAM, 128 CUDA cores, ARM64 architecture

echo "=========================================="
echo "Enhanced Camera-LiDAR Fusion System"
echo "Jetson Nano Optimized Version"
echo "=========================================="

# Set display for Jetson Nano
export DISPLAY=:0

# Performance optimizations for Jetson Nano
echo "Applying Jetson Nano performance optimizations..."

# Set CPU governor to performance mode
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Set GPU governor to performance mode
echo performance | sudo tee /sys/devices/gpu.0/devfreq/17000000.gp10b/governor

# Increase swap space if needed (for 4GB RAM constraint)
SWAP_SIZE=$(free | grep Swap | awk '{print $2}')
if [ $SWAP_SIZE -lt 2000000 ]; then
    echo "Warning: Low swap space detected. Consider increasing swap for better performance."
fi

# Check available memory
FREE_MEM=$(free -m | grep Mem | awk '{print $7}')
echo "Available memory: ${FREE_MEM}MB"

if [ $FREE_MEM -lt 1000 ]; then
    echo "Warning: Low memory available. Consider closing other applications."
fi

# Check GPU memory
GPU_MEM=$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits | head -1)
echo "Available GPU memory: ${GPU_MEM}MB"

# Set environment variables for better performance
export CUDA_VISIBLE_DEVICES=0
export CUDA_LAUNCH_BLOCKING=0
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
    sed -i 's/\/dev\/ttyUSB0/\/dev\/ttyUSB1/g' $(find . -name "enhanced_system_jetson.launch")
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

# Check if required packages are installed
echo "Checking required packages..."

# Check for scikit-learn (required for enhanced filtering)
python3 -c "import sklearn" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "Installing scikit-learn..."
    pip3 install scikit-learn
fi

# Check for psutil (required for performance monitoring)
python3 -c "import psutil" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "Installing psutil..."
    pip3 install psutil
fi

# Check for opencv-python
python3 -c "import cv2" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "Installing opencv-python..."
    pip3 install opencv-python
fi

# Make scripts executable
echo "Making scripts executable..."
chmod +x src/camera_lidar_fusion/scripts/*.py

# Create performance log directory
mkdir -p ~/camera_lidar_logs

# Launch the enhanced system
echo "Launching Enhanced Camera-LiDAR Fusion System..."
echo "This may take a few moments to initialize..."

# Launch with reduced priority to prevent system overload
nice -n 10 roslaunch camera_lidar_fusion enhanced_system_jetson.launch

# If launch fails, try with minimal configuration
if [ $? -ne 0 ]; then
    echo "Enhanced system launch failed. Trying minimal configuration..."
    echo "This will run with reduced features for better compatibility."
    
    # Launch minimal system
    nice -n 10 roslaunch camera_lidar_fusion complete_system_final.launch
fi

echo "System shutdown complete."
