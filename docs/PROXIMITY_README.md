# Camera+LiDAR Proximity System for ArduPilot

This system provides real-time proximity data from Camera+LiDAR fusion to ArduPilot via MAVROS, enabling Mission Planner's Proximity view to display live obstacle detection data.

## System Overview

The system consists of:
1. **Camera+LiDAR Fusion**: Processes ZED stereo camera and UniLiDAR data
2. **PointCloud to LaserScan Converter**: Converts 3D point clouds to 2D laser scans
3. **Scan to Rangefinder Bridge**: Converts laser scans to MAVROS-compatible messages
4. **MAVROS**: Bridges ROS messages to ArduPilot via MAVLink

## Topics

### Input
- `/unilidar/cloud` (sensor_msgs/PointCloud2): Raw LiDAR point cloud
- `/scan` (sensor_msgs/LaserScan): Converted 2D laser scan

### Output
- `/mavros/distance_sensor/rangefinder_pub` (sensor_msgs/Range): Single closest distance for rangefinder
- `/mavros/obstacle/send` (sensor_msgs/LaserScan): 72-bin proximity data for Mission Planner
- `/mavros/obstacle/distances` (std_msgs/Float32MultiArray): All binned distances

## How to Run

### 1. Start the Complete System
```bash
cd ~/CameraLidar/catkin_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
roslaunch mavprox_bridge bridge_distance_sensor_final.launch
```

### 2. Verify System Status
```bash
# Check MAVROS connection
rostopic echo -n1 /mavros/state

# Check rangefinder data (should be ~10Hz)
rostopic hz /mavros/distance_sensor/rangefinder_pub

# Check proximity data (should be ~10Hz)
rostopic hz /mavros/obstacle/send

# Check data values
rostopic echo -n1 /mavros/obstacle/send
```

## ArduPilot Configuration

### Required Parameters (set in Mission Planner)
- `PRX1_TYPE = 5` (MAVLink proximity)
- `RNGFND_TYPE = 1` (Analog)
- `RNGFND_SCALING = 100`
- `RNGFND_MIN_CM = 10`
- `RNGFND_MAX_CM = 5000`
- `RNGFND_ORIENT = 25` (Forward)
- `RNGFND_OFFSET = 0`
- `RNGFND_FUNCTION = 0`

### Optional Parameters
- `AVOID_ENABLE = 0` (disable obstacle avoidance if not needed)

## Expected Performance

- **Update Rate**: ~10Hz for both rangefinder and proximity data
- **Range**: 0.1m to 50.0m
- **Resolution**: 72 bins (5° each, 360° coverage)
- **Frame**: base_link (x forward, y right, z down)

## Troubleshooting

### No Data in Mission Planner
1. **Check MAVROS Connection**:
   ```bash
   rostopic echo -n1 /mavros/state
   # Should show: connected: True
   ```

2. **Check Data Flow**:
   ```bash
   rostopic hz /mavros/obstacle/send
   # Should show ~10Hz
   ```

3. **Check ArduPilot Parameters**:
   - Verify `PRX1_TYPE = 5` is set
   - Check Mission Planner Messages tab for errors

4. **Check Sensor Data**:
   ```bash
   rostopic echo -n1 /scan
   # Should show valid laser scan data
   ```

### No Rangefinder Data
1. **Check rangefinder topic**:
   ```bash
   rostopic hz /mavros/distance_sensor/rangefinder_pub
   ```

2. **Check ArduPilot rangefinder parameters**:
   - `RNGFND_TYPE = 1`
   - `RNGFND_SCALING = 100`

### System Not Starting
1. **Check ROS master**:
   ```bash
   roscore &
   ```

2. **Check hardware connections**:
   - ZED camera connected
   - UniLiDAR connected to /dev/ttyUSB0
   - Pixhawk connected to /dev/ttyTHS1

3. **Check node status**:
   ```bash
   rosnode list
   rosnode info /scan_to_rangefinder
   ```

## Test Publisher

For testing without the full system, use the test publisher:
```bash
python ~/prox_test_obstacle.py
```

This publishes synthetic LaserScan data to `/mavros/obstacle/send` at 5Hz.

## Architecture

```
UniLiDAR → PointCloud2 → LaserScan → ScanToRangefinder → MAVROS → ArduPilot → Mission Planner
   ↓           ↓           ↓              ↓              ↓         ↓           ↓
/dev/ttyUSB0  /unilidar/cloud  /scan  /mavros/obstacle/send  MAVLink  Proximity View
```

## Files

- `scan_to_rangefinder.cpp`: Main bridge node
- `pointcloud_to_laserscan_simple.cpp`: PointCloud to LaserScan converter
- `bridge_distance_sensor_final.launch`: Main launch file
- `mavros_distance_config.yaml`: MAVROS configuration
- `prox_test_obstacle.py`: Test publisher

## Support

For issues:
1. Check ROS logs: `rosnode info /scan_to_rangefinder`
2. Check MAVROS logs: `rosnode info /mavros`
3. Verify hardware connections
4. Check ArduPilot parameters in Mission Planner

