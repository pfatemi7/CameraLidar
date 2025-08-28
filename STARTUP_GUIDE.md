# 🚁 Quick Startup Guide

## **Every Time You Want to Use the System:**

### **Step 1: Physical Connections** ✅
1. **Connect Pixhawk** to Jetson via USB cable
2. **Connect ZED Camera** to Jetson USB port
3. **Connect UniLiDAR** to Jetson USB port
4. **Power on Jetson**

### **Step 2: Start the System** 🚀

**Option A: One-Command Start (Recommended)**
```bash
# SSH to Jetson
ssh skyscouter@10.42.0.213

# Run the startup script
cd ~/CameraLidar/scripts
./start_system_complete.sh
```

**Option B: Manual Start**
```bash
# SSH to Jetson
ssh skyscouter@10.42.0.213

# Navigate to workspace
cd ~/CameraLidar/catkin_ws

# Source ROS
source /opt/ros/melodic/setup.bash
source devel/setup.bash

# Start the system
roslaunch camera_lidar_fusion complete_system_final.launch
```

### **Step 3: Mission Planner Setup** 📱
1. **Open Mission Planner**
2. **Connect to your Pixhawk**
3. **Go to Flight Data → Proximity**
4. **See live 360° proximity data!** 🎉

## **What You Should See:**

### **In Terminal:**
- ✅ ROS master started
- ✅ MAVROS connected to ArduPilot
- ✅ Proximity data publishing (~10Hz)
- ✅ LiDAR data publishing
- ✅ Camera data publishing

### **In Mission Planner:**
- **Flight Data → Proximity**: Live 360° proximity visualization
- **72 distance bins** around the drone
- **Real-time updates** as objects move

## **Troubleshooting:**

### **If MAVROS not connected:**
- Check USB connection to Pixhawk
- Verify Pixhawk is powered on
- Check `/dev/ttyTHS1` exists

### **If no proximity data:**
- Check UniLiDAR USB connection
- Verify `PRX1_TYPE = 5` in Mission Planner
- Check sensor power

### **If no camera data:**
- Check ZED camera USB connection
- Verify camera is powered

## **Stop the System:**
```bash
# In terminal: Ctrl+C
# Or kill all ROS processes:
pkill -f roslaunch
pkill -f roscore
```

## **Monitor System:**
```bash
# List all topics
rostopic list

# Check proximity data rate
rostopic hz /mavros/obstacle/send

# View proximity data
rostopic echo /mavros/obstacle/send
```

---

**That's it! One command starts everything!** 🎉
