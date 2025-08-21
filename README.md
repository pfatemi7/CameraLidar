# Camera-LiDAR Fusion for NVIDIA Jetson

This project provides a complete ROS-based camera-LiDAR fusion system designed for NVIDIA Jetson devices. It combines data from a ZED stereo camera and a UniLiDAR sensor to create colored point clouds and perform sensor fusion.

## Features

- **Real-time sensor fusion** between ZED camera and LiDAR
- **Point cloud filtering** with voxel downsampling, Z-axis filtering, and statistical outlier removal
- **Color mapping** of LiDAR points using camera data
- **Calibration tools** for camera-LiDAR extrinsic calibration
- **Visualization** with RViz and debug images
- **Both C++ and Python implementations** for flexibility

## System Requirements

### Hardware
- NVIDIA Jetson device (Xavier NX, AGX Xavier, etc.)
- ZED stereo camera
- UniLiDAR sensor
- USB 3.0 ports for sensors

### Software
- Ubuntu 20.04 LTS
- ROS Noetic
- ZED SDK for Tegra
- UniLiDAR SDK
- OpenCV 4.x
- PCL (Point Cloud Library)
- Python 3.8+

## Installation

### 1. Prerequisites

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full

# Install dependencies
sudo apt install python3-pip python3-catkin-tools
sudo apt install ros-noetic-pcl-ros ros-noetic-pcl-conversions
sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport
sudo apt install ros-noetic-message-filters ros-noetic-tf2-ros
sudo apt install ros-noetic-rviz ros-noetic-rqt-plot

# Install Python dependencies
pip3 install numpy opencv-python pcl
```

### 2. Install ZED SDK

```bash
# Download and install ZED SDK for Tegra
# Follow instructions at: https://www.stereolabs.com/docs/installation/linux/
# Make sure to install the Tegra version compatible with your Jetson
```

### 3. Install UniLiDAR SDK

```bash
# Follow the UniLiDAR SDK installation instructions
# This will depend on your specific LiDAR model
```

### 4. Clone and Build

```bash
# Clone the repository
git clone <your-repo-url> CameraLidar
cd CameraLidar

# Build the workspace
cd catkin_ws
catkin build
source devel/setup.bash
```

## Usage

### 1. Basic Setup

First, ensure your sensors are connected and recognized:

```bash
# Check ZED camera
lsusb | grep ZED

# Check LiDAR (device name may vary)
ls /dev/ttyUSB*

# Set permissions for LiDAR
sudo chmod 666 /dev/ttyUSB0
```

### 2. Calibration

Before running the fusion system, you need to calibrate the camera-LiDAR transformation:

```bash
# Start calibration
roslaunch camera_lidar_fusion calibration.launch
```

The calibration process:
1. Press 'c' to enter calibration mode
2. Adjust the transformation parameters in the code or use the calibration tools
3. Press 's' to save the calibration
4. Press 'r' to reset if needed

### 3. Running the Fusion System

```bash
# Start the main fusion system
roslaunch camera_lidar_fusion camera_lidar_fusion.launch
```

This will:
- Start the camera-LiDAR fusion node
- Launch RViz for visualization
- Start RQT plot for monitoring

### 4. Topics

The system publishes the following topics:

- `/fused/cloud` - Filtered point cloud
- `/fused/colored_cloud` - Colored point cloud
- `/fused/debug_image` - Debug image showing projected points
- `/tf` - Transform between camera and LiDAR frames

### 5. Parameters

You can adjust the following parameters in the launch files:

- `voxel_leaf_size` - Voxel grid filter leaf size (default: 0.03m)
- `z_min`, `z_max` - Z-axis filter limits (default: -0.2m to 2.5m)
- `sor_mean_k` - Statistical outlier removal mean K (default: 20)
- `sor_std_dev` - Statistical outlier removal standard deviation (default: 1.0)

## Development Workflow

### On Your Laptop (Development)

1. **Write and test code** using Cursor
2. **Commit changes** to git:
   ```bash
   git add .
   git commit -m "Your commit message"
   git push origin main
   ```

### On Jetson (Deployment)

**Option A: Use the setup script (Recommended)**
```bash
# Pull the latest code
cd ~/CameraLidar
git pull origin main

# Run the minimal setup script (recommended for Jetson)
chmod +x setup_jetson_minimal.sh
./setup_jetson_minimal.sh

# Or run the full setup script if you need all features
# chmod +x setup_jetson_fixed.sh
# ./setup_jetson_fixed.sh
```

**Option B: Manual setup**
```bash
# Pull the latest code
cd ~/CameraLidar
git pull origin main

# Install catkin-tools
sudo apt install python3-catkin-tools

# Build the workspace
cd catkin_ws
catkin build
source devel/setup.bash
```

3. **Run the system**:
   ```bash
   # For systems with PCL Python bindings
   roslaunch camera_lidar_fusion camera_lidar_fusion.launch
   
   # For systems without PCL Python bindings (recommended for Jetson)
   roslaunch camera_lidar_fusion camera_lidar_fusion_simple.launch
   ```

## Troubleshooting

### Common Issues

1. **Permission denied for LiDAR**:
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   ```

2. **ZED camera not detected**:
   - Check USB connection
   - Verify ZED SDK installation
   - Check camera permissions

3. **Build errors**:
   ```bash
   # Clean and rebuild
   cd catkin_ws
   catkin clean
   catkin build
   
   # If catkin build not found, install catkin-tools
   sudo apt install python3-catkin-tools
   
   # Or use catkin_make as alternative
   catkin_make
   ```

4. **Python import errors**:
   ```bash
   # Install missing Python packages
   pip3 install <package-name>
   ```

5. **Package not found errors**:
   ```bash
   # Update package list
   sudo apt update
   
   # Try minimal setup instead
   ./setup_jetson_minimal.sh
   
   # Or install packages manually
   sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport
   ```

### Performance Optimization

For better performance on Jetson:

1. **Reduce point cloud density** by increasing `voxel_leaf_size`
2. **Limit processing range** by adjusting `z_min` and `z_max`
3. **Use the C++ version** for better performance
4. **Enable GPU acceleration** where possible

## File Structure

```
CameraLidar/
├── catkin_ws/
│   └── src/
│       ├── camera_lidar_fusion/
│       │   ├── src/
│       │   │   └── camera_lidar_fusion_node.cpp
│       │   ├── scripts/
│       │   │   ├── camera_lidar_fusion_node.py
│       │   │   ├── calibration_node.py
│       │   │   └── pcl_helper.py
│       │   ├── launch/
│       │   │   ├── camera_lidar_fusion.launch
│       │   │   └── calibration.launch
│       │   ├── config/
│       │   │   ├── camera_lidar_fusion.rviz
│       │   │   └── calibration.rviz
│       │   ├── package.xml
│       │   └── CMakeLists.txt
│       ├── filters_launch/
│       │   └── launch/
│       │       └── l1_filters.launch
│       └── unilidar_sdk/
├── LICENSE
└── README.md
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test on both laptop and Jetson
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Support

For issues and questions:
1. Check the troubleshooting section
2. Review ROS and sensor documentation
3. Open an issue on the repository
