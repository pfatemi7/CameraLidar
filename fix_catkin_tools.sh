#!/bin/bash

echo "🔧 Fixing catkin-tools Python compatibility issues..."

# Fix catkin-tools Python compatibility
echo "📦 Installing importlib-metadata for Python 3.6 compatibility..."
pip3 install importlib-metadata

# Alternative: Reinstall catkin-tools with system Python
echo "🔄 Reinstalling catkin-tools..."
sudo apt remove python3-catkin-tools -y
sudo apt install python3-catkin-tools -y

# Or use catkin_make instead
echo "💡 If catkin build still fails, use catkin_make:"
echo "   cd ~/CameraLidar/catkin_ws"
echo "   catkin_make"
echo "   source devel/setup.bash"

echo "✅ Fix attempt complete!"
