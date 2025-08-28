# Proximity Visualization System

This system provides comprehensive visualization and validation tools for Camera+LiDAR fusion proximity detection on Unitree L1 drones with ArduPilot integration.

## 🎯 Features

- **Orientation Correction**: Configurable TF transforms for LiDAR mounting corrections
- **360° Proximity Ring**: Real-time visualization of filtered proximity data
- **Point Cloud Visualization**: Transformed LiDAR data in base frame
- **RViz Integration**: Complete visualization setup with optimized configuration
- **Mission Planner Validation**: Verify proximity data matches between RViz and Mission Planner

## 📁 File Structure

```
mavprox_bridge/
├── viz/
│   ├── visualize_proximity.py      # Main visualization node
│   └── l1_tf_publisher.py          # TF transform utility
├── launch/
│   └── proximity_viz.launch        # Complete system launch file
├── config/
│   └── proximity_viz.yaml          # Configuration parameters
├── rviz/
│   └── proximity_check.rviz        # RViz configuration
└── setup.py                        # Python script installation
```

## 🚀 Quick Start

### 1. Build the Package

```bash
cd ~/CameraLidar/catkin_ws
source /opt/ros/melodic/setup.bash
catkin_make
source devel/setup.bash
```

### 2. Run the Visualization System

```bash
roslaunch mavprox_bridge proximity_viz.launch
```

This will start:
- L1 TF Publisher (orientation correction)
- Proximity Visualizer (360° ring visualization)
- RViz with optimized configuration

## ⚙️ Configuration

### Basic Configuration (`config/proximity_viz.yaml`)

```yaml
# Frame Configuration
base_frame: base_link
l1_frame: l1_link

# Topic Configuration
lidar_topic: /unilidar/cloud
prox_topic: /mavros/obstacle/send

# TF Transform Parameters
x_m: 0.0          # X offset (meters)
y_m: 0.0          # Y offset (meters)
z_m: 0.0          # Z offset (meters)
roll_deg: 0.0     # Roll angle (degrees)
pitch_deg: 0.0    # Pitch angle (degrees)
yaw_deg: 0.0      # Yaw angle (degrees) - KEY for alignment

# Axis Inversions
invert_x: false   # Invert X axis
invert_y: false   # Invert Y axis
invert_z: false   # Invert Z axis

# Visualization Parameters
min_range: 1.0    # Minimum range (meters)
max_range: 5.0    # Maximum range (meters)
sector_width_deg: 5.0  # Sector width (degrees)
```

### Common LiDAR Orientations

#### LiDAR Mounted Upside Down
```yaml
invert_z: true
pitch_deg: 180.0
```

#### LiDAR Mounted Sideways (90°)
```yaml
yaw_deg: 90.0
```

#### LiDAR with Offset Position
```yaml
x_m: 0.1    # 10cm forward
y_m: 0.0    # No lateral offset
z_m: -0.05  # 5cm down
```

## 🔧 Tuning Instructions

### Step 1: Align LiDAR Orientation

1. **Start the system**:
   ```bash
   roslaunch mavprox_bridge proximity_viz.launch
   ```

2. **Check raw point cloud** in RViz:
   - Look at the `L1 Cloud Base` display
   - Verify points are visible and oriented correctly
   - If not, adjust `yaw_deg`, `roll_deg`, `pitch_deg` parameters

3. **Common adjustments**:
   - **Front not pointing forward**: Adjust `yaw_deg`
   - **Upside down**: Set `invert_z: true` and `pitch_deg: 180.0`
   - **Sideways**: Set `yaw_deg: 90.0` or `yaw_deg: -90.0`

### Step 2: Verify Proximity Ring Alignment

1. **Place a test object** ~2.5m directly in front of the drone

2. **Check RViz visualization**:
   - The proximity ring should show a red sector at the correct bearing
   - The point cloud should show returns at +X in base_link frame

3. **Check Mission Planner**:
   - Go to Flight Data → Proximity
   - Verify the same sector is lit in Mission Planner

### Step 3: Validate Rotation Consistency

1. **Rotate the vehicle 90°** (either physically or in simulation)

2. **Verify consistency**:
   - RViz proximity ring should rotate with the vehicle
   - Mission Planner proximity should rotate consistently
   - If not aligned, adjust `yaw_deg` parameter

## 🧪 Validation Steps

### Test 1: Front Obstacle Detection

1. Place a box ~2.5m directly in front
2. Verify:
   - ✅ RViz point cloud shows returns at +X in base_link
   - ✅ Proximity ring shows red sector at correct bearing
   - ✅ Mission Planner Proximity shows same sector lit

### Test 2: Side Obstacle Detection

1. Place a box ~2.5m to the right side
2. Verify:
   - ✅ RViz shows red sector at +Y (90°)
   - ✅ Mission Planner shows corresponding sector

### Test 3: Rotation Test

1. Rotate vehicle 90° clockwise
2. Verify:
   - ✅ RViz proximity ring rotates with vehicle
   - ✅ Mission Planner proximity rotates consistently
   - ✅ Front obstacle now appears at -Y (270°)

### Test 4: Range Validation

1. Place obstacles at different distances:
   - 0.5m (should be ignored - outside 1.0-5.0m range)
   - 1.5m (should show green/yellow)
   - 3.0m (should show yellow)
   - 4.5m (should show red)
   - 6.0m (should be ignored - outside range)

## 🔍 Troubleshooting

### Issue: No Point Cloud Visible
- **Check**: LiDAR topic is publishing
- **Solution**: Verify `/unilidar/cloud` topic exists and has data
- **Command**: `rostopic hz /unilidar/cloud`

### Issue: Proximity Ring Not Appearing
- **Check**: Proximity topic is publishing
- **Solution**: Verify `/mavros/obstacle/send` topic has data
- **Command**: `rostopic hz /mavros/obstacle/send`

### Issue: Misaligned Orientation
- **Check**: TF transform is correct
- **Solution**: Adjust `yaw_deg`, `roll_deg`, `pitch_deg` parameters
- **Command**: `rosrun tf tf_echo base_link l1_link`

### Issue: RViz Not Loading
- **Check**: RViz configuration file exists
- **Solution**: Verify `rviz/proximity_check.rviz` is present
- **Alternative**: Start RViz manually and load the config

### Issue: Python Scripts Not Found
- **Check**: Scripts are installed
- **Solution**: Rebuild the workspace
- **Command**: `catkin_make && source devel/setup.bash`

## 📊 Visualization Details

### Proximity Ring Colors
- **Green**: Far obstacles (4-5m) - Safe
- **Yellow**: Medium obstacles (2-4m) - Caution
- **Red**: Near obstacles (1-2m) - Danger
- **Gray**: Invalid readings (transparent)

### Range Rings
- **Inner ring**: 1.0m (minimum range)
- **Outer ring**: 5.0m (maximum range)

### Sector Information
- **72 sectors**: 5° each (360° total)
- **Real-time updates**: Based on filtered proximity data
- **Frame**: All visualization in `base_link` frame

## 🔗 Topic Mapping

| Component | Topic | Message Type | Description |
|-----------|-------|--------------|-------------|
| LiDAR Input | `/unilidar/cloud` | `PointCloud2` | Raw LiDAR data |
| Proximity Input | `/mavros/obstacle/send` | `LaserScan` | Filtered proximity data |
| Transformed Cloud | `/proximity_visualizer/l1_cloud_base` | `PointCloud2` | LiDAR in base frame |
| Proximity Markers | `/proximity_visualizer/proximity_markers` | `MarkerArray` | 360° ring visualization |

## 🎛️ Advanced Configuration

### Custom Topics
```yaml
lidar_topic: /custom/lidar/topic
prox_topic: /custom/proximity/topic
```

### Custom Frames
```yaml
base_frame: custom_base_frame
l1_frame: custom_lidar_frame
```

### Visualization Tuning
```yaml
alpha_valid: 0.9        # Valid reading transparency
alpha_invalid: 0.15     # Invalid reading transparency
show_invalid: true      # Show invalid readings
republish_cloud: true   # Republish transformed cloud
```

## 📝 Notes

- **Python 3**: All scripts target Python 3 for compatibility
- **TF2**: Uses modern TF2 for transformations
- **Efficient**: Marker reuse and cleanup for performance
- **Robust**: Handles missing data and transformation errors gracefully
- **Configurable**: All parameters exposed via ROS parameters

## 🆘 Support

For issues or questions:
1. Check the troubleshooting section above
2. Verify all topics are publishing: `rostopic list`
3. Check TF transforms: `rosrun tf tf_echo base_link l1_link`
4. Review RViz console for error messages
