# MAVROS Integration Summary

## What Has Been Implemented

### 1. MAVROS Obstacle Avoidance Node (`mavros_obstacle_node.py`)
- **Purpose**: Converts camera-LiDAR fusion data into MAVROS obstacle distance messages
- **Input**: Point cloud data from `/fused/cloud` and object detection distances
- **Output**: `mavros_msgs/ObstacleDistance` messages to `/mavros/obstacle/send`
- **Features**:
  - 180° horizontal FOV with configurable angle increments
  - 60° vertical FOV filtering
  - Distance measurements in centimeters (0 = unknown)
  - Real-time processing of point cloud data
  - Configurable maximum distance (default: 30m)

### 2. Velocity Controller Node (`velocity_controller_node.py`)
- **Purpose**: Optional velocity control for guided flight based on obstacle detection
- **Input**: Obstacle distance messages and point cloud data
- **Output**: `geometry_msgs/TwistStamped` to `/mavros/setpoint_velocity/cmd_vel`
- **Features**:
  - Automatic obstacle avoidance behavior
  - Configurable safety distance and maximum velocity
  - Smooth velocity transitions
  - Yaw rate control for turning away from obstacles

### 3. MAVROS Integration Launch File (`mavros_integration.launch`)
- **Purpose**: Complete system launch with MAVROS and camera-LiDAR fusion
- **Components**:
  - MAVROS node with ArduPilot configuration
  - Complete camera-LiDAR fusion system
  - Obstacle avoidance node
  - Optional velocity controller
  - RViz visualization

### 4. Test Node (`test_mavros_integration.py`)
- **Purpose**: Test MAVROS integration without real sensors
- **Features**:
  - Publishes simulated obstacle data
  - Test obstacles at +45° (5m) and -45° (3m)
  - 1 Hz update rate for testing

### 5. RViz Configuration (`mavros_integration.rviz`)
- **Purpose**: Visualization of the complete system
- **Displays**:
  - Grid and coordinate frames
  - Raw LiDAR point cloud
  - Fused point cloud
  - Debug camera image
  - TF transforms

## System Architecture

```
SITL (ArduCopter on Laptop)
    ↕ UDP (14550)
Mission Planner (Laptop GUI)
    ↕ UDP (14555)
MAVROS (Jetson Nano)
    ↕ ROS Topics
Camera-LiDAR Fusion System
    ↕ /mavros/obstacle/send
ArduPilot Obstacle Avoidance
```

## Quick Start Guide

### Prerequisites
1. **Laptop**: ArduPilot SITL running
2. **Jetson**: ROS Melodic, MAVROS, camera-LiDAR fusion system
3. **Network**: Jetson accessible at `10.42.0.213`

### Step 1: Start SITL on Laptop
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --console --map

# In MAVProxy console:
output add 127.0.0.1:14550
output add 10.42.0.213:14555
```

### Step 2: Test MAVROS Integration (Optional)
```bash
# On Jetson - Test without real sensors
cd ~/CameraLidar/catkin_ws
source devel/setup.bash
roslaunch camera_lidar_fusion test_mavros.launch
```

### Step 3: Start Full System
```bash
# On Jetson - Full camera-LiDAR fusion with MAVROS
cd ~/CameraLidar/catkin_ws
source devel/setup.bash
roslaunch camera_lidar_fusion mavros_integration.launch
```

### Step 4: Verify Data Flow
```bash
# Terminal 1: Check MAVROS connection
rostopic echo /mavros/state

# Terminal 2: Monitor obstacle data
rostopic echo /mavros/obstacle/send

# Terminal 3: Check fusion output
rostopic echo /object_detection/distance
```

## Configuration Parameters

### Obstacle Avoidance Parameters
```xml
<param name="fov_horizontal" value="180.0" />  <!-- degrees -->
<param name="fov_vertical" value="60.0" />      <!-- degrees -->
<param name="max_distance" value="30.0" />      <!-- meters -->
<param name="angle_increment" value="2.0" />    <!-- degrees -->
```

### Velocity Control Parameters
```xml
<param name="max_velocity" value="2.0" />       <!-- m/s -->
<param name="safety_distance" value="5.0" />    <!-- meters -->
<param name="enable_avoidance" value="true" />  <!-- enable/disable -->
```

## Testing and Verification

### 1. Basic Functionality Test
```bash
# Start test system
roslaunch camera_lidar_fusion test_mavros.launch

# Check obstacle messages
rostopic echo /mavros/obstacle/send -n 1
```

### 2. Mission Planner Verification
1. Connect Mission Planner to `127.0.0.1:14550`
2. Go to Flight Data → Obstacle Avoidance
3. Verify obstacle distance array is updating

### 3. SITL Flight Test
```bash
# In MAVProxy console:
mode GUIDED
arm throttle
takeoff 10
# Send waypoints or use guided mode
```

## Troubleshooting

### Common Issues

1. **MAVROS not connecting**:
   ```bash
   rostopic echo /mavros/state
   # Check network connectivity and UDP ports
   ```

2. **No obstacle data**:
   ```bash
   rostopic echo /fused/cloud
   # Verify camera-LiDAR fusion is working
   ```

3. **Wrong distances**:
   - Check LiDAR calibration
   - Verify sensor mounting and orientation

4. **Build errors**:
   ```bash
   cd ~/CameraLidar/catkin_ws
   source /opt/ros/melodic/setup.bash
   catkin_make
   ```

### Debug Commands
```bash
# Check all topics
rostopic list

# Monitor message rates
rostopic hz /mavros/obstacle/send

# Check node status
rosnode list
rosnode info /mavros_obstacle_node
```

## File Structure

```
camera_lidar_fusion/
├── scripts/
│   ├── mavros_obstacle_node.py      # Main obstacle avoidance
│   ├── velocity_controller_node.py  # Optional velocity control
│   ├── test_mavros_integration.py   # Test node
│   └── calibrated_fusion_node.py    # Camera-LiDAR fusion
├── launch/
│   ├── mavros_integration.launch    # Complete system
│   └── test_mavros.launch          # Test system
├── config/
│   └── mavros_integration.rviz      # RViz configuration
├── package.xml                      # Updated dependencies
└── README_MAVROS_INTEGRATION.md     # Detailed documentation
```

## Next Steps

1. **Test with SITL**: Verify obstacle avoidance works in simulation
2. **Real Hardware**: Adapt for actual drone hardware
3. **Advanced Features**: Implement more sophisticated avoidance algorithms
4. **Mission Integration**: Test with autonomous missions
5. **Performance Tuning**: Optimize for your specific use case

## Support

For issues:
1. Check the troubleshooting section
2. Verify all prerequisites are installed
3. Test individual components separately
4. Check ROS and MAVROS logs
5. Ensure network connectivity between laptop and Jetson

## Safety Notes

- **Always test in simulation first**
- **Start with conservative parameters**
- **Monitor system performance**
- **Have manual override capability**
- **Test in open areas initially**
