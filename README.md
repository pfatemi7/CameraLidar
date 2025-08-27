# 🚁 Camera-LiDAR Fusion with ArduPilot Integration

A complete ROS-based system that fuses ZED stereo camera and UniLiDAR data, providing real-time proximity detection and obstacle avoidance for ArduPilot-powered drones.

## 🌟 Features

- **Real-time Camera-LiDAR Fusion**: Combines ZED stereo camera and UniLiDAR sensor data
- **ArduPilot Integration**: Live proximity data in Mission Planner
- **360° Obstacle Detection**: 72-bin proximity array around the drone
- **ROS Melodic Support**: Full ROS ecosystem integration
- **Jetson Optimized**: Designed for NVIDIA Jetson devices
- **Mission Planner Compatible**: Live proximity visualization

## 📋 System Requirements

### Hardware
- **NVIDIA Jetson** (TX2, Xavier, or newer)
- **ZED Stereo Camera** (ZED2i recommended)
- **UniLiDAR Sensor** (360° LiDAR)
- **Pixhawk Flight Controller** (ArduPilot)
- **WiFi/Network Connection** for MAVROS

### Software
- **Ubuntu 18.04** (for ROS Melodic)
- **ROS Melodic**
- **ArduPilot** (latest stable)
- **Mission Planner** (for ground control)

## 🚀 Quick Start

### 1. Clone the Repository
```bash
git clone https://github.com/yourusername/camera-lidar-fusion.git
cd camera-lidar-fusion
```

### 2. Install Dependencies
```bash
# Install ROS Melodic (if not already installed)
./scripts/install_ros_melodic_jetson.sh

# Build the workspace
./scripts/build_workspace.sh
```

### 3. Configure ArduPilot
In Mission Planner:
1. Connect to your Pixhawk
2. Go to **Config/Tuning → Full Parameter List**
3. Set `PRX1_TYPE = 5` (MAVLink proximity)
4. Click **"Write Params"** and reboot

### 4. Start the System
```bash
# SSH to Jetson and start the system
./scripts/run_prox_demo.sh
```

### 5. View Proximity Data
1. Open Mission Planner
2. Connect to your Pixhawk
3. Go to **Flight Data → Proximity**
4. See live 360° proximity data! 🎉

## 📁 Project Structure

```
camera-lidar-fusion/
├── catkin_ws/                          # ROS workspace
│   └── src/
│       ├── camera_lidar_fusion/        # Main fusion package
│       ├── mavprox_bridge/             # MAVROS bridge package
│       ├── unitree_lidar_ros/          # LiDAR driver
│       └── zed-ros-wrapper/            # ZED camera driver
├── configs/                            # Configuration files
│   ├── mavros_distance_config.yaml    # MAVROS plugin config
│   └── *.launch                        # ROS launch files
├── docs/                               # Documentation
│   └── PROXIMITY_README.md            # Detailed proximity guide
├── scripts/                            # Utility scripts
│   ├── run_prox_demo.sh               # One-click system start
│   ├── install_ros_melodic_jetson.sh  # ROS installation
│   └── build_workspace.sh             # Workspace build
└── README.md                          # This file
```

## 🔧 Key Components

### 1. Camera-LiDAR Fusion (`camera_lidar_fusion`)
- **Enhanced Filtering Node**: Processes raw sensor data
- **Object Detection & Tracking**: YOLO-based object detection
- **Performance Monitoring**: Real-time system metrics
- **Distance Monitoring**: Safety distance alerts

### 2. MAVROS Bridge (`mavprox_bridge`)
- **PointCloud to LaserScan**: Converts LiDAR point clouds
- **Scan to Rangefinder**: Bridges to ArduPilot
- **Distance Sensor Plugin**: MAVROS integration
- **Obstacle Distance**: Multi-sector proximity data

### 3. Sensor Drivers
- **UniLiDAR ROS Driver**: 360° LiDAR support
- **ZED ROS Wrapper**: Stereo camera integration

## 📊 System Performance

- **Update Rate**: ~10Hz proximity data
- **Detection Range**: 0.1m to 50m
- **Angular Resolution**: 5° (72 bins)
- **Latency**: <100ms end-to-end
- **CPU Usage**: <30% on Jetson TX2

## 🎯 Use Cases

- **Autonomous Navigation**: Obstacle avoidance
- **Indoor Flight**: Close proximity detection
- **Search & Rescue**: Object detection and tracking
- **Inspection**: Visual and distance data fusion
- **Research**: Multi-sensor fusion studies

## 🔍 Troubleshooting

### Common Issues

1. **No Proximity Data in Mission Planner**
   - Check `PRX1_TYPE = 5` parameter
   - Verify MAVROS connection
   - Ensure system is running on Jetson

2. **LiDAR Detection Range Limited**
   - Check Z-axis filters in fusion nodes
   - Verify LiDAR calibration
   - Check for obstructions

3. **High CPU Usage**
   - Reduce camera resolution
   - Disable unused nodes
   - Check for background processes

### Debug Commands
```bash
# Check system status
./scripts/run_prox_demo.sh

# Monitor topics
rostopic hz /mavros/obstacle/send
rostopic echo /mavros/state

# Check MAVROS connection
rosservice call /mavros/param/get 'param_id: "PRX1_TYPE"'
```

## 📚 Documentation

- **[Proximity Integration Guide](docs/PROXIMITY_README.md)**: Detailed setup instructions
- **[Jetson Setup Guide](JETSON_SETUP_COMPLETE.md)**: Complete Jetson configuration
- **[ROS Package Documentation](catkin_ws/src/camera_lidar_fusion/README.md)**: Package-specific docs

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **ArduPilot Team**: For the excellent flight controller software
- **ZED SDK**: For stereo camera integration
- **ROS Community**: For the robotics middleware
- **UniLiDAR**: For the LiDAR sensor support

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/yourusername/camera-lidar-fusion/issues)
- **Discussions**: [GitHub Discussions](https://github.com/yourusername/camera-lidar-fusion/discussions)
- **Wiki**: [Project Wiki](https://github.com/yourusername/camera-lidar-fusion/wiki)

---

**Made with ❤️ for the drone community**

*Happy flying! 🛸✨*
