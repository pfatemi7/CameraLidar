#!/bin/bash

# Camera+LiDAR Proximity Demo Script
# This script starts the complete proximity system and verifies it's working

set -e  # Exit on any error

echo "🚁 Camera+LiDAR Proximity System Demo"
echo "======================================"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if we're in the right directory
if [ ! -d "catkin_ws" ]; then
    print_error "Please run this script from the CameraLidar directory"
    exit 1
fi

cd catkin_ws

# Source ROS environment
print_status "Sourcing ROS environment..."
source /opt/ros/melodic/setup.bash
source devel/setup.bash

# Check if ROS master is running
if ! rostopic list >/dev/null 2>&1; then
    print_warning "ROS master not running, starting roscore..."
    roscore &
    sleep 3
fi

# Check if MAVROS is already running
if rostopic list | grep -q "/mavros/state"; then
    print_warning "MAVROS appears to be running already"
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Start the complete system
print_status "Starting Camera+LiDAR proximity system..."
roslaunch mavprox_bridge bridge_distance_sensor_final.launch &
LAUNCH_PID=$!

# Wait for system to start
print_status "Waiting for system to initialize..."
sleep 10

# Function to check if a topic is publishing
check_topic() {
    local topic=$1
    local expected_rate=$2
    local description=$3
    
    print_status "Checking $description..."
    
    if ! rostopic list | grep -q "$topic"; then
        print_error "Topic $topic not found!"
        return 1
    fi
    
    # Check if topic is publishing
    local rate=$(timeout 5 rostopic hz "$topic" 2>/dev/null | grep "average rate" | head -1 | awk '{print $3}' | sed 's/,//')
    
    if [ -z "$rate" ] || [ "$rate" = "0.000" ]; then
        print_error "Topic $topic is not publishing data!"
        return 1
    fi
    
    print_status "$description: ✓ Publishing at ${rate}Hz"
    return 0
}

# Check MAVROS connection
print_status "Checking MAVROS connection..."
if ! timeout 5 rostopic echo -n1 /mavros/state >/dev/null 2>&1; then
    print_error "Cannot read /mavros/state - MAVROS may not be connected"
    echo "Please check:"
    echo "1. Pixhawk is connected to /dev/ttyTHS1"
    echo "2. MAVROS is properly configured"
    exit 1
fi

MAVROS_STATE=$(timeout 5 rostopic echo -n1 /mavros/state 2>/dev/null | grep "connected:" | awk '{print $2}')
if [ "$MAVROS_STATE" = "True" ]; then
    print_status "MAVROS: ✓ Connected to ArduPilot"
else
    print_warning "MAVROS: ⚠ Not connected to ArduPilot"
fi

# Check all required topics
ERRORS=0

check_topic "/scan" "10" "LaserScan data" || ERRORS=$((ERRORS + 1))
check_topic "/mavros/distance_sensor/rangefinder_pub" "10" "Rangefinder data" || ERRORS=$((ERRORS + 1))
check_topic "/mavros/obstacle/send" "10" "Proximity data" || ERRORS=$((ERRORS + 1))

# Check data quality
print_status "Checking data quality..."

# Check rangefinder data
RANGEFINDER_DATA=$(timeout 3 rostopic echo -n1 /mavros/distance_sensor/rangefinder_pub 2>/dev/null | grep "range:" | awk '{print $2}')
if [ -n "$RANGEFINDER_DATA" ] && [ "$RANGEFINDER_DATA" != "0.0" ]; then
    print_status "Rangefinder: ✓ Valid data (${RANGEFINDER_DATA}m)"
else
    print_warning "Rangefinder: ⚠ No valid data"
    ERRORS=$((ERRORS + 1))
fi

# Check proximity data
PROXIMITY_RANGES=$(timeout 3 rostopic echo -n1 /mavros/obstacle/send 2>/dev/null | grep "ranges:" | wc -w)
if [ "$PROXIMITY_RANGES" -gt 70 ]; then
    print_status "Proximity: ✓ Valid data (${PROXIMITY_RANGES} ranges)"
else
    print_warning "Proximity: ⚠ Insufficient data (${PROXIMITY_RANGES} ranges)"
    ERRORS=$((ERRORS + 1))
fi

# Summary
echo
echo "======================================"
if [ $ERRORS -eq 0 ]; then
    print_status "🎉 System is working correctly!"
    echo
    echo "Next steps:"
    echo "1. Open Mission Planner"
    echo "2. Connect to your Pixhawk"
    echo "3. Set PRX1_TYPE = 5 in Full Parameter List"
    echo "4. Go to Flight Data → Proximity"
    echo "5. You should see live proximity data!"
else
    print_error "❌ System has $ERRORS issue(s)"
    echo
    echo "Troubleshooting:"
    echo "1. Check hardware connections"
    echo "2. Verify ArduPilot parameters"
    echo "3. Check ROS logs: rosnode info /scan_to_rangefinder"
    echo "4. Check MAVROS logs: rosnode info /mavros"
fi

echo
echo "System is running in background (PID: $LAUNCH_PID)"
echo "To stop: kill $LAUNCH_PID"
echo "To monitor: rostopic hz /mavros/obstacle/send"

