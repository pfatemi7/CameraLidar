# Camera-LiDAR Fusion System - FINAL VERSION

## 🎯 **System Overview**

This is a complete camera-LiDAR fusion system that:
- **Detects objects** using camera motion detection
- **Calculates distances** using real LiDAR data
- **Provides alerts** when objects are too close
- **Visualizes everything** in RViz and image viewers

## 📋 **Hardware Setup**

### **Required Hardware:**
- NVIDIA Jetson (Ubuntu 18.04 + ROS Melodic)
- ZED Camera
- UniLiDAR (mounted 90° from ground, pointing forward like camera)
- Monitor connected to Jetson

### **Physical Setup:**
1. **ZED Camera**: Mounted normally, pointing forward
2. **UniLiDAR**: Mounted 90° from ground, pointing forward (same direction as camera)
3. **LiDAR Offset**: 10cm above camera (0.1m vertical offset)
4. **Horizontal Alignment**: LiDAR and camera centers aligned

## 🚀 **Quick Start (Tomorrow)**

### **Method 1: Single Command (Recommended)**
```bash
cd ~/CameraLidar/clean_ws/src/camera_lidar_fusion/scripts
./start_system.sh
```

### **Method 2: Manual Launch**
```bash
cd ~/CameraLidar/clean_ws
source devel/setup.bash
export DISPLAY=:0
roslaunch camera_lidar_fusion complete_system_final.launch
```

### **Method 3: Individual Components (for debugging)**
```bash
# Terminal 1: ROS Core
cd ~/CameraLidar/clean_ws
source devel/setup.bash
roscore

# Terminal 2: ZED Camera
cd ~/CameraLidar/clean_ws
source devel/setup.bash
roslaunch zed_wrapper zed.launch

# Terminal 3: UniLiDAR
cd ~/CameraLidar/clean_ws
source devel/setup.bash
roslaunch unitree_lidar_ros run.launch

# Terminal 4: Complete System
cd ~/CameraLidar/clean_ws
source devel/setup.bash
export DISPLAY=:0
roslaunch camera_lidar_fusion complete_system_final.launch
```

## 📊 **What You'll See**

### **On Jetson Monitor:**
1. **RViz**: 3D visualization with point clouds and camera feed
2. **Image Viewers**: Left and right camera feeds
3. **Terminal Output**: Distance alerts and system status

### **System Features:**
- **Object Detection**: Green boxes around detected objects
- **Real LiDAR Distances**: "Person: 1.23m (LiDAR)" labels
- **Distance Alerts**: 
  - **Warning**: Objects within 90cm
  - **Danger**: Objects within 50cm
- **Web Monitor**: Access via browser (if needed)

## 🔧 **Troubleshooting**

### **If LiDAR Not Working:**
```bash
# Check LiDAR connection
ls /dev/ttyUSB*

# Test LiDAR data
cd ~/CameraLidar/clean_ws
source devel/setup.bash
rosrun camera_lidar_fusion lidar_distance_test.py
```

### **If Camera Not Working:**
```bash
# Check camera connection
ls /dev/video*

# Test camera
cd ~/CameraLidar/clean_ws
source devel/setup.bash
rosrun camera_lidar_fusion minimal_test_node.py
```

### **If RViz Not Opening:**
```bash
# Set display
export DISPLAY=:0

# Or run with X11 forwarding
ssh -X skyscouter@10.42.0.213
```

## 📁 **File Structure**

```
catkin_ws/src/camera_lidar_fusion/
├── launch/
│   └── complete_system_final.launch    # Main launch file
├── scripts/
│   ├── start_system.sh                 # Startup script
│   ├── calibrated_fusion_node.py       # Main fusion node
│   ├── distance_monitor_node.py        # Distance alerts
│   ├── lidar_distance_test.py          # LiDAR test
│   ├── minimal_test_node.py            # Camera test
│   └── web_monitor.py                  # Web interface
└── config/
    └── simple_fusion.rviz              # RViz configuration
```

## 🎯 **Testing the System**

### **Basic Test:**
1. **Start the system** using any method above
2. **Walk in front of camera** at 1 meter distance
3. **Check RViz** for green bounding box and distance label
4. **Move closer** to test distance alerts

### **Distance Accuracy Test:**
1. **Place object at exactly 1 meter**
2. **Check if distance shows ~1.00m**
3. **Move to 0.5m** - should trigger danger alert
4. **Move to 0.9m** - should trigger warning alert

## ⚙️ **Configuration**

### **Distance Thresholds:**
- **Warning**: 90cm (0.9m)
- **Danger**: 50cm (0.5m)

### **Calibration Parameters:**
- **LiDAR Offset**: (0.0, 0.0, 0.1m)
- **Camera FOV**: 90° x 60°
- **LiDAR FOV**: 179.9° x 179.9°

## 🛑 **Stopping the System**

### **Single Command Method:**
Press `Ctrl+C` in the terminal

### **Individual Components:**
Press `Ctrl+C` in each terminal

## 📞 **Support**

If you encounter issues:
1. **Check hardware connections** (camera, LiDAR, monitor)
2. **Verify LiDAR orientation** (pointing forward, not up)
3. **Run individual test nodes** to isolate problems
4. **Check terminal output** for error messages

## 🎉 **Success Indicators**

✅ **System is working when you see:**
- RViz showing point clouds and camera feed
- Green bounding boxes around detected objects
- Real distance measurements (e.g., "Person: 1.23m (LiDAR)")
- Distance alerts in terminal when objects are close
- Image viewers showing camera feeds

**Your Camera-LiDAR Fusion System is ready!** 🚀
