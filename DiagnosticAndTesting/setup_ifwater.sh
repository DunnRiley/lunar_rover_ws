#!/bin/bash
# IFWATER Camera Complete Setup Script

echo "========================================"
echo "  IFWATER Camera Setup & Installation  "
echo "========================================"
echo ""

# Step 1: Install required ROS2 package
echo "Step 1: Installing usb_cam ROS2 package..."
echo ""
sudo apt update
sudo apt install -y ros-humble-usb-cam ros-humble-image-transport-plugins

if [ $? -eq 0 ]; then
    echo "✓ usb_cam installed successfully"
else
    echo "✗ Failed to install usb_cam"
    echo "Try manually: sudo apt install ros-humble-usb-cam"
    exit 1
fi
echo ""

# Step 2: Install ffmpeg for testing
echo "Step 2: Installing ffmpeg (for camera testing)..."
echo ""
sudo apt install -y ffmpeg v4l-utils

if [ $? -eq 0 ]; then
    echo "✓ ffmpeg and v4l-utils installed"
else
    echo "⚠ Optional tools installation failed (not critical)"
fi
echo ""

# Step 3: Add user to video group
echo "Step 3: Adding user to video group..."
echo ""
sudo usermod -aG video $USER

if [ $? -eq 0 ]; then
    echo "✓ User added to video group"
    echo "  NOTE: You may need to log out and log back in for this to take effect"
else
    echo "⚠ Failed to add user to video group"
fi
echo ""

# Step 4: Detect camera
echo "Step 4: Detecting IFWATER camera..."
echo ""

echo "Before plugging in camera, current video devices:"
ls /dev/video* 2>/dev/null | wc -l
echo ""

echo "Please ensure your IFWATER camera is plugged in"
echo "Press Enter when ready..."
read

echo ""
echo "After camera plugged in, video devices:"
ls /dev/video* 2>/dev/null
echo ""

echo "Looking for most recently created devices..."
ls -lt /dev/video* | head -5
echo ""

# Try to identify IFWATER
echo "Scanning for USB cameras (not Intel IPU6)..."
for device in /dev/video32 /dev/video33 /dev/video2 /dev/video3 /dev/video4 /dev/video5; do
    if [ -c "$device" ]; then
        info=$(v4l2-ctl --device=$device --info 2>/dev/null | grep "Card type")
        if echo "$info" | grep -v -q "ipu6"; then
            echo "  Found non-IPU6 device: $device"
            echo "    $info"
        fi
    fi
done
echo ""

# Step 5: Test camera
echo "========================================"
echo "Step 5: Testing camera..."
echo "========================================"
echo ""

echo "Which video device is your LEFT camera? (e.g., /dev/video32)"
read LEFT_DEVICE

echo "Which video device is your RIGHT camera? (e.g., /dev/video33)"
read RIGHT_DEVICE

echo ""
echo "Testing LEFT camera: $LEFT_DEVICE"
if v4l2-ctl --device=$LEFT_DEVICE --list-formats-ext 2>/dev/null | grep -q "1920x1080"; then
    echo "✓ $LEFT_DEVICE supports 1920x1080"
    
    # Test capture
    echo "  Capturing test image..."
    if ffmpeg -f v4l2 -video_size 1920x1080 -i $LEFT_DEVICE -frames:v 1 /tmp/ifwater_left_test.jpg -y 2>/dev/null; then
        echo "  ✓ Test image saved to /tmp/ifwater_left_test.jpg"
    else
        echo "  ⚠ Could not capture at 1920x1080, trying 640x480..."
        ffmpeg -f v4l2 -video_size 640x480 -i $LEFT_DEVICE -frames:v 1 /tmp/ifwater_left_test.jpg -y 2>/dev/null
    fi
else
    echo "✗ $LEFT_DEVICE does not support 1920x1080"
    echo "Available resolutions:"
    v4l2-ctl --device=$LEFT_DEVICE --list-formats-ext 2>/dev/null | grep "Size:" | head -10
fi
echo ""

echo "Testing RIGHT camera: $RIGHT_DEVICE"
if v4l2-ctl --device=$RIGHT_DEVICE --list-formats-ext 2>/dev/null | grep -q "1920x1080"; then
    echo "✓ $RIGHT_DEVICE supports 1920x1080"
    
    echo "  Capturing test image..."
    if ffmpeg -f v4l2 -video_size 1920x1080 -i $RIGHT_DEVICE -frames:v 1 /tmp/ifwater_right_test.jpg -y 2>/dev/null; then
        echo "  ✓ Test image saved to /tmp/ifwater_right_test.jpg"
    else
        echo "  ⚠ Could not capture at 1920x1080, trying 640x480..."
        ffmpeg -f v4l2 -video_size 640x480 -i $RIGHT_DEVICE -frames:v 1 /tmp/ifwater_right_test.jpg -y 2>/dev/null
    fi
else
    echo "✗ $RIGHT_DEVICE does not support 1920x1080"
    echo "Available resolutions:"
    v4l2-ctl --device=$RIGHT_DEVICE --list-formats-ext 2>/dev/null | grep "Size:" | head -10
fi
echo ""

# Step 6: Test ROS2 integration
echo "========================================"
echo "Step 6: Testing ROS2 integration..."
echo "========================================"
echo ""

echo "Launching LEFT camera in ROS2 for 5 seconds..."
timeout 5 ros2 run usb_cam usb_cam_node_exe --ros-args \
    -p video_device:=$LEFT_DEVICE \
    -p image_width:=1920 \
    -p image_height:=1080 \
    -p framerate:=30.0 &

sleep 2

echo ""
echo "Checking if camera topic is publishing..."
if ros2 topic list | grep -q "/image_raw"; then
    echo "✓ Camera is publishing to ROS2!"
    echo ""
    echo "Topic info:"
    ros2 topic info /image_raw
    echo ""
    echo "Checking publish rate..."
    timeout 3 ros2 topic hz /image_raw
else
    echo "✗ Camera topic not found"
fi

# Kill the camera node
pkill -f usb_cam_node_exe

echo ""
echo "========================================"
echo "SETUP COMPLETE!"
echo "========================================"
echo ""
echo "Your IFWATER camera devices:"
echo "  LEFT:  $LEFT_DEVICE"
echo "  RIGHT: $RIGHT_DEVICE"
echo ""
echo "Save this configuration file:"
cat > ~/lunar_rover_ws/ifwater_config.txt << EOF
# IFWATER Camera Configuration
LEFT_CAMERA=$LEFT_DEVICE
RIGHT_CAMERA=$RIGHT_DEVICE
EOF

echo ""
echo "Configuration saved to: ~/lunar_rover_ws/ifwater_config.txt"
echo ""
echo "NEXT STEPS:"
echo "1. Update rover_launcher.py with these device paths"
echo "2. Or use the GUI 'Configure Camera Devices' button"
echo ""
echo "Test individual cameras with:"
echo "  ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=$LEFT_DEVICE"
echo ""