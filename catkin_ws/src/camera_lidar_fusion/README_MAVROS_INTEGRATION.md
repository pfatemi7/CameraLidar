# Camera-LiDAR Fusion with MAVROS Integration

This document describes how to integrate the Camera-LiDAR fusion system with MAVROS for ArduPilot obstacle avoidance in SITL (Software In The Loop) simulation.

## Overview

The system integrates:
- **ZED Camera**: For object detection and visual feedback
- **UniLiDAR**: For 3D point cloud data
- **Camera-LiDAR Fusion**: Combines sensor data for accurate obstacle detection
- **MAVROS**: ROS interface to ArduPilot
- **ArduPilot SITL**: Software simulation of the autopilot

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

## Prerequisites

### On Laptop (Development Machine)
- ArduPilot SITL installed
- Mission Planner installed
- Git access to the repository

### On Jetson Nano
- ROS Melodic installed
- MAVROS installed (`ros-melodic-mavros`)
- Camera-LiDAR fusion system working
- Network connectivity to laptop

## Installation

### 1. Build the Workspace

On the Jetson Nano:

```bash
cd ~/CameraLidar/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. Verify MAVROS Installation

```bash
# Check if MAVROS is installed
dpkg -l | grep mavros

# Should show:
# ros-melodic-mavros
# ros-melodic-mavros-extras
# ros-melodic-mavros-msgs
```

If not installed, install it:

```bash
sudo apt update
sudo apt install ros-melodic-mavros ros-melodic-mavros-extras ros-melodic-mavros-msgs
```

## Usage

### 1. Start SITL on Laptop

```bash
# Terminal 1: Start ArduPilot SITL
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --console --map

# In MAVProxy console, set up forwarding:
output add 127.0.0.1:14550
output add 10.42.0.213:14555
```

### 2. Start Mission Planner (Optional)

Open Mission Planner and connect to `127.0.0.1:14550` to monitor the simulation.

### 3. Start MAVROS Integration on Jetson

```bash
# Terminal 2: Start the complete system
cd ~/CameraLidar/catkin_ws
source devel/setup.bash
roslaunch camera_lidar_fusion mavros_integration.launch
```

This will start:
- MAVROS node connected to SITL
- ZED camera node
- UniLiDAR node
- Camera-LiDAR fusion node
- MAVROS obstacle avoidance node
- RViz visualization

### 4. Verify Data Flow

Check that data is flowing correctly:

```bash
# Terminal 3: Monitor obstacle data
rostopic echo /mavros/obstacle/send

# Terminal 4: Monitor MAVROS state
rostopic echo /mavros/state

# Terminal 5: Monitor fusion output
rostopic echo /object_detection/distance
```

## Configuration

### MAVROS Parameters

Edit `mavros_integration.launch` to modify:

```xml
<!-- MAVROS Configuration -->
<arg name="fcu_url" default="udp://:14555@" />
<arg name="tgt_system" default="1" />
<arg name="tgt_component" default="1" />
```

### Obstacle Avoidance Parameters

```xml
<!-- Obstacle avoidance parameters -->
<param name="fov_horizontal" value="180.0" />  <!-- degrees -->
<param name="fov_vertical" value="60.0" />      <!-- degrees -->
<param name="max_distance" value="30.0" />      <!-- meters -->
<param name="angle_increment" value="2.0" />    <!-- degrees -->
```

### Velocity Control Parameters (Optional)

```xml
<!-- Velocity control parameters -->
<param name="max_velocity" value="2.0" />       <!-- m/s -->
<param name="safety_distance" value="5.0" />    <!-- meters -->
```

## Testing

### 1. Basic Obstacle Detection

1. Start the system as described above
2. Move objects in front of the camera-LiDAR setup
3. Check `/mavros/obstacle/send` topic for distance measurements
4. Verify in Mission Planner that obstacle avoidance is active

### 2. SITL Flight Test

1. In MAVProxy console, arm the vehicle:
   ```
   mode GUIDED
   arm throttle
   ```

2. Take off:
   ```
   takeoff 10
   ```

3. Send waypoints or use guided mode to test obstacle avoidance:
   ```
   wp add 0 0 10
   wp add 10 0 10
   mode AUTO
   ```

### 3. Mission Planner Verification

In Mission Planner:
1. Go to Flight Data → Obstacle Avoidance
2. You should see the obstacle distance array being updated
3. The avoidance system should react to obstacles

## Troubleshooting

### MAVROS Connection Issues

```bash
# Check if MAVROS is receiving data
rostopic echo /mavros/state

# Check MAVROS logs
rosnode info /mavros
```

### Obstacle Data Issues

```bash
# Check if obstacle messages are being published
rostopic hz /mavros/obstacle/send

# Check message content
rostopic echo /mavros/obstacle/send -n 1
```

### Camera-LiDAR Issues

```bash
# Check if fusion is working
rostopic echo /fused/cloud

# Check object detection
rostopic echo /object_detection/distance
```

### Common Issues

1. **No obstacle data**: Check if point cloud is being published
2. **Wrong distances**: Verify LiDAR calibration
3. **MAVROS not connecting**: Check network connectivity and UDP ports
4. **SITL not responding**: Restart SITL and check MAVProxy forwarding

## Advanced Features

### Velocity Control

Enable velocity control for guided flight:

```bash
roslaunch camera_lidar_fusion mavros_integration.launch enable_velocity_control:=true
```

This will publish velocity setpoints to `/mavros/setpoint_velocity/cmd_vel`.

### Custom Obstacle Avoidance

Modify `mavros_obstacle_node.py` to implement custom avoidance algorithms:

- Different FOV configurations
- Multiple sensor fusion
- Advanced filtering algorithms

### Mission Integration

Integrate with ArduPilot missions:

1. Create a mission in Mission Planner
2. Enable obstacle avoidance in mission parameters
3. Upload and execute the mission
4. Monitor avoidance behavior

## File Structure

```
camera_lidar_fusion/
├── scripts/
│   ├── mavros_obstacle_node.py      # Main obstacle avoidance node
│   ├── velocity_controller_node.py  # Optional velocity control
│   └── calibrated_fusion_node.py    # Camera-LiDAR fusion
├── launch/
│   └── mavros_integration.launch    # Complete system launch
├── config/
│   └── mavros_integration.rviz      # RViz configuration
└── package.xml                      # Updated with MAVROS dependencies
```

## Performance Tuning

### Obstacle Detection Sensitivity

Adjust parameters in `mavros_obstacle_node.py`:

```python
self.fov_horizontal = 180.0    # Wider FOV for better coverage
self.angle_increment = 1.0     # Higher resolution (more measurements)
self.max_distance = 20.0       # Shorter range for faster response
```

### Update Rates

```python
self.publish_rate = rospy.Rate(20)  # Higher rate for smoother avoidance
```

### Filtering

Add additional filtering in the point cloud processing:

```python
# Add noise filtering
if distance < 0.1:  # Ignore very close points
    continue
```

## Safety Considerations

1. **Always test in simulation first**
2. **Start with conservative parameters**
3. **Monitor system performance**
4. **Have manual override capability**
5. **Test in open areas initially**

## Support

For issues and questions:
1. Check the troubleshooting section above
2. Verify all prerequisites are installed
3. Check ROS and MAVROS logs
4. Test individual components separately

## Next Steps

1. **Real Hardware Integration**: Adapt for real drone hardware
2. **Advanced Algorithms**: Implement more sophisticated avoidance
3. **Multi-Sensor Fusion**: Add additional sensors
4. **Machine Learning**: Integrate ML-based object detection
5. **Mission Planning**: Develop autonomous mission capabilities
