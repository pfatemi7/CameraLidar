#!/bin/bash

# Script to run RViz on local machine connected to Jetson ROS master
# This allows visualization of proximity data from Jetson

echo "🚀 Starting RViz on local machine connected to Jetson..."

# Set ROS master to point to Jetson
export ROS_MASTER_URI=http://10.42.0.213:11311

# Set ROS IP to local machine
export ROS_IP=$(hostname -I | awk '{print $1}')

echo "📡 Connecting to Jetson ROS master at: $ROS_MASTER_URI"
echo "🖥️  Local ROS IP: $ROS_IP"

# Check if we can reach the Jetson
echo "🔍 Testing connection to Jetson..."
if ping -c 1 10.42.0.213 > /dev/null 2>&1; then
    echo "✅ Jetson is reachable"
else
    echo "❌ Cannot reach Jetson at 10.42.0.213"
    echo "Please check network connection and Jetson IP address"
    exit 1
fi

# Check if ROS topics are available
echo "📊 Checking ROS topics from Jetson..."
if timeout 5 rostopic list > /dev/null 2>&1; then
    echo "✅ ROS topics available from Jetson"
else
    echo "❌ Cannot access ROS topics from Jetson"
    echo "Make sure the Jetson is running ROS and the proximity system"
    exit 1
fi

# Start RViz with the proximity configuration
echo "🎯 Starting RViz with proximity visualization..."
rviz -d $(rospack find mavprox_bridge)/rviz/proximity_check.rviz

echo "🏁 RViz closed"
