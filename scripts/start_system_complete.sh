#!/bin/bash

# 🚁 Camera-LiDAR Fusion System - Complete Startup Script
# This script starts the entire system with one command

echo "🚁 Starting Camera-LiDAR Fusion System with ArduPilot Integration..."
echo "================================================================"

# Check if we're on Jetson
if [[ $(hostname) == *"jetson"* ]] || [[ $(hostname) == *"skyscouter"* ]]; then
    echo "✅ Detected Jetson device"
    JETSON_MODE=true
else
    echo "❌ This script should be run on the Jetson device"
    echo "   Please SSH to Jetson and run this script there"
    exit 1
fi

# Navigate to workspace
cd ~/CameraLidar/catkin_ws

# Source ROS
echo "📦 Sourcing ROS Melodic..."
source /opt/ros/melodic/setup.bash
source devel/setup.bash

# Kill any existing ROS processes
echo "🧹 Cleaning up existing ROS processes..."
pkill -f roscore
pkill -f roslaunch
sleep 2

# Start ROS master
echo "🎯 Starting ROS master..."
roscore &
ROS_PID=$!
sleep 3

# Check if roscore started successfully
if ! pgrep -x "roscore" > /dev/null; then
    echo "❌ Failed to start roscore"
    exit 1
fi
echo "✅ ROS master started"

# Start the complete system
echo "🚀 Launching complete Camera-LiDAR fusion system..."
roslaunch camera_lidar_fusion complete_system_final.launch &
LAUNCH_PID=$!

# Wait for system to initialize
echo "⏳ Waiting for system to initialize..."
sleep 10

# Check system status
echo "🔍 Checking system status..."

# Check if MAVROS is connected
echo "📡 Checking MAVROS connection..."
timeout 5 rostopic echo -n1 /mavros/state > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✅ MAVROS connected to ArduPilot"
else
    echo "⚠️  MAVROS not connected - check Pixhawk connection"
fi

# Check proximity data
echo "📊 Checking proximity data..."
timeout 5 rostopic hz /mavros/obstacle/send > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✅ Proximity data publishing"
else
    echo "⚠️  Proximity data not publishing - check sensors"
fi

# Check LiDAR data
echo "🔍 Checking LiDAR data..."
timeout 5 rostopic echo -n1 /unilidar/cloud > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✅ LiDAR data publishing"
else
    echo "⚠️  LiDAR data not publishing - check UniLiDAR connection"
fi

# Check camera data
echo "📷 Checking camera data..."
timeout 5 rostopic echo -n1 /zed/left/image_rect_color > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✅ Camera data publishing"
else
    echo "⚠️  Camera data not publishing - check ZED camera connection"
fi

echo ""
echo "🎉 System startup complete!"
echo "================================================================"
echo "📋 Next steps:"
echo "1. Open Mission Planner"
echo "2. Connect to your Pixhawk"
echo "3. Go to Flight Data → Proximity"
echo "4. You should see live 360° proximity data!"
echo ""
echo "🔧 To stop the system: Ctrl+C or run 'pkill -f roslaunch'"
echo "📊 To monitor topics: 'rostopic list' or 'rostopic hz /mavros/obstacle/send'"
echo ""

# Keep the script running
echo "🔄 System running... Press Ctrl+C to stop"
wait $LAUNCH_PID
