# 🚀 Quick Start Guide

Get your Camera-LiDAR fusion system with ArduPilot integration running in 5 minutes!

## ⚡ Super Quick Setup

### 1. Clone & Setup
```bash
git clone https://github.com/pfatemi7/CameraLidar.git
cd CameraLidar
chmod +x scripts/*.sh
```

### 2. Install ROS (Jetson)
```bash
./scripts/install_ros_melodic_jetson.sh
```

### 3. Build Workspace
```bash
./scripts/build_workspace.sh
```

### 4. Configure ArduPilot
In Mission Planner:
- Connect to Pixhawk
- Set `PRX1_TYPE = 5`
- Write params & reboot

### 5. Start System
```bash
./scripts/run_prox_demo.sh
```

### 6. View Proximity Data
- Open Mission Planner
- Go to **Flight Data → Proximity**
- See live 360° data! 🎉

## 🔧 What You Get

✅ **Real-time proximity detection** (0.1m - 50m)  
✅ **360° coverage** (72 bins, 5° resolution)  
✅ **10Hz update rate**  
✅ **Mission Planner integration**  
✅ **Complete documentation**  

## 🆘 Need Help?

- **Full Documentation**: [README.md](../README.md)
- **Proximity Guide**: [PROXIMITY_README.md](PROXIMITY_README.md)
- **Troubleshooting**: Check the main README

## 🎯 System Status

Your system is now ready for:
- Autonomous navigation
- Obstacle avoidance
- Indoor flight
- Search & rescue
- Research projects

**Happy flying! 🛸✨**
