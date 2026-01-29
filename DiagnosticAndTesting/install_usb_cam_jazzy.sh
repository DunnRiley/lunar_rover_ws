#!/bin/bash
# Install usb_cam for ROS2 Jazzy on Ubuntu 24.04

echo "========================================"
echo "  Installing usb_cam for ROS2 Jazzy    "
echo "========================================"
echo ""

echo "Attempting to install from apt repositories..."
sudo apt update

# Try to install from Jazzy repos
sudo apt install -y ros-jazzy-usb-cam

if [ $? -eq 0 ]; then
    echo "✓ usb_cam installed successfully from apt!"
    echo ""
    echo "Test with:"
    echo "  source /opt/ros/jazzy/setup.bash"
    echo "  ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video32"
    exit 0
fi

echo ""
echo "⚠ Package not found in apt, building from source..."
echo ""

# If apt fails, build from source
cd ~/lunar_rover_ws
mkdir -p src
cd src

echo "Installing dependencies..."
sudo apt install -y \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libavutil-dev \
    libv4l-dev \
    v4l-utils \
    ffmpeg

echo ""
echo "Cloning usb_cam repository..."
if [ -d "usb_cam" ]; then
    rm -rf usb_cam
fi

git clone https://github.com/ros-drivers/usb_cam.git -b ros2

echo ""
echo "Building..."
cd ~/lunar_rover_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select usb_cam --symlink-install

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ usb_cam built successfully!"
    echo ""
    echo "Add to ~/.bashrc:"
    echo "  echo 'source ~/lunar_rover_ws/install/setup.bash' >> ~/.bashrc"
    echo ""
    echo "Test with:"
    echo "  source ~/lunar_rover_ws/install/setup.bash"
    echo "  ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video32"
else
    echo ""
    echo "✗ Build failed"
    echo ""
    echo "Use the simple_camera_publisher.py instead:"
    echo "  python3 simple_camera_publisher.py"
fi