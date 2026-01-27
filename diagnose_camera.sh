#!/bin/bash
# Camera Diagnostic Script
# Finds your RealSense camera and shows proper device info

echo "========================================"
echo "  RealSense Camera Diagnostic          "
echo "========================================"
echo ""

echo "1. Checking for video devices..."
if ls /dev/video* 2>/dev/null; then
    echo ""
    echo "2. Checking which are RealSense cameras..."
    for device in /dev/video*; do
        info=$(v4l2-ctl --device=$device --info 2>/dev/null | grep -i "realsense\|intel" || echo "")
        if [ -n "$info" ]; then
            echo "  ✓ Found RealSense at: $device"
            v4l2-ctl --device=$device --info 2>/dev/null | head -5
            echo ""
        fi
    done
else
    echo "  ✗ No video devices found!"
    echo ""
    echo "Possible issues:"
    echo "  - Camera not plugged in"
    echo "  - Camera not powered"
    echo "  - USB port issue"
    echo "  - Need to run: sudo ./fix_camera_usb.sh"
    exit 1
fi

echo ""
echo "3. Checking USB connection..."
if lsusb | grep -i "intel\|realsense"; then
    echo "  ✓ RealSense camera detected on USB"
else
    echo "  ✗ No RealSense camera on USB"
fi

echo ""
echo "4. Checking if realsense2_camera package is installed..."
if ros2 pkg list 2>/dev/null | grep -q realsense2_camera; then
    echo "  ✓ realsense2_camera package found"
else
    echo "  ✗ realsense2_camera package NOT installed!"
    echo "  Install with: sudo apt install ros-humble-realsense2-camera"
fi

echo ""
echo "5. Testing simple camera launch..."
echo "   (This will run for 5 seconds to test)"
echo ""

timeout 5 ros2 launch realsense2_camera rs_launch.py 2>&1 | head -20 || echo "Launch test complete"

echo ""
echo "========================================"
echo "  Diagnostic Complete                   "
echo "========================================"
echo ""
echo "If camera was found, you can use:"
echo "  bash test_camera_transforms.sh"
echo ""
echo "If camera issues persist, try:"
echo "  sudo ./fix_camera_usb.sh"