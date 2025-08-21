#!/bin/bash

echo "🧪 Testing Jetson Camera-LiDAR Fusion Setup..."

# Connect to Jetson and test the setup
ssh -i ~/.ssh/jetson_key skyscouter@10.42.0.213 << 'EOF'
cd ~/CameraLidar/clean_ws
source devel/setup.bash

echo "✅ Package found: $(rospack find camera_lidar_fusion)"
echo "✅ Launch files available:"
ls src/camera_lidar_fusion/launch/
echo "✅ Scripts available:"
ls src/camera_lidar_fusion/scripts/

echo ""
echo "🚀 To run the system:"
echo "1. Start roscore: roscore"
echo "2. In another terminal: cd ~/CameraLidar/clean_ws && source devel/setup.bash"
echo "3. Run test node: rosrun camera_lidar_fusion simple_test_node.py"
echo "4. Or run full system: roslaunch camera_lidar_fusion camera_lidar_fusion_simple.launch"
EOF

echo "✅ Test complete!"
