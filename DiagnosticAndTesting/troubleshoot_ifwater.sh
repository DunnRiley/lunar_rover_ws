#!/bin/bash
# IFWATER Camera Troubleshooting Script

echo "========================================"
echo "  IFWATER Camera Troubleshooter        "
echo "========================================"
echo ""

echo "Step 1: Checking USB devices..."
echo ""
lsusb | grep -i "camera\|webcam\|usb\|video" || echo "  No obvious camera devices found in USB"
echo ""
echo "Full USB device list:"
lsusb
echo ""

echo "========================================" 
echo "Step 2: Looking for NEW video devices..."
echo "  (These should be /dev/video32 and /dev/video33)"
echo "========================================" 
echo ""

# The IFWATER camera was just plugged in - it created video32 and video33
echo "Recently created devices (most likely your IFWATER):"
ls -lt /dev/video* | head -10
echo ""

echo "Testing /dev/video32 (likely IFWATER left):"
v4l2-ctl --device=/dev/video32 --info 2>/dev/null || echo "  ✗ Cannot access /dev/video32"
echo ""

echo "Testing /dev/video33 (likely IFWATER right):"
v4l2-ctl --device=/dev/video33 --info 2>/dev/null || echo "  ✗ Cannot access /dev/video33"
echo ""

echo "========================================"
echo "Step 3: Checking supported formats..."
echo "========================================"
echo ""

if v4l2-ctl --device=/dev/video32 --list-formats-ext 2>/dev/null | grep -q "1920x1080"; then
    echo "✓ /dev/video32 supports 1920x1080"
    echo "  Supported formats:"
    v4l2-ctl --device=/dev/video32 --list-formats-ext 2>/dev/null | grep -E "Pixel Format|Size: " | head -20
else
    echo "✗ /dev/video32 does not support 1920x1080 or cannot be accessed"
fi
echo ""

if v4l2-ctl --device=/dev/video33 --list-formats-ext 2>/dev/null | grep -q "1920x1080"; then
    echo "✓ /dev/video33 supports 1920x1080"
    echo "  Supported formats:"
    v4l2-ctl --device=/dev/video33 --list-formats-ext 2>/dev/null | grep -E "Pixel Format|Size: " | head -20
else
    echo "✗ /dev/video33 does not support 1920x1080 or cannot be accessed"
fi
echo ""

echo "========================================"
echo "Step 4: Checking ROS2 usb_cam package"
echo "========================================"
echo ""

if ros2 pkg list 2>/dev/null | grep -q usb_cam; then
    echo "✓ usb_cam package is installed"
else
    echo "✗ usb_cam package NOT installed!"
    echo ""
    echo "Install with:"
    echo "  sudo apt update"
    echo "  sudo apt install ros-humble-usb-cam"
    echo ""
fi

echo "========================================"
echo "Step 5: Testing camera access"
echo "========================================"
echo ""

echo "Attempting to capture a test frame from /dev/video32..."
if ffmpeg -f v4l2 -video_size 640x480 -i /dev/video32 -frames:v 1 /tmp/test_left.jpg -y 2>/dev/null; then
    echo "✓ Successfully captured test frame from /dev/video32"
    echo "  Image saved to: /tmp/test_left.jpg"
else
    echo "✗ Failed to capture from /dev/video32"
    echo ""
    echo "Possible issues:"
    echo "  - Camera not plugged in"
    echo "  - Wrong device path"
    echo "  - Permission issues"
    echo "  - ffmpeg not installed (install with: sudo apt install ffmpeg)"
fi
echo ""

echo "Attempting to capture a test frame from /dev/video33..."
if ffmpeg -f v4l2 -video_size 640x480 -i /dev/video33 -frames:v 1 /tmp/test_right.jpg -y 2>/dev/null; then
    echo "✓ Successfully captured test frame from /dev/video33"
    echo "  Image saved to: /tmp/test_right.jpg"
else
    echo "✗ Failed to capture from /dev/video33"
fi
echo ""

echo "========================================"
echo "DIAGNOSIS SUMMARY"
echo "========================================"
echo ""

# Check if IFWATER is detected
if [ -c /dev/video32 ] && [ -c /dev/video33 ]; then
    echo "✓ IFWATER camera devices detected (/dev/video32, /dev/video33)"
    echo ""
    echo "RECOMMENDED DEVICE PATHS:"
    echo "  Left camera:  /dev/video32"
    echo "  Right camera: /dev/video33"
    echo ""
    
    # Check if usb_cam is installed
    if ros2 pkg list 2>/dev/null | grep -q usb_cam; then
        echo "✓ usb_cam package is ready"
        echo ""
        echo "NEXT STEPS:"
        echo "  1. Use these devices in your launcher:"
        echo "     - Left:  /dev/video32"
        echo "     - Right: /dev/video33"
        echo ""
        echo "  2. Test with:"
        echo "     ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video32"
    else
        echo "✗ usb_cam package missing"
        echo ""
        echo "INSTALL REQUIRED PACKAGE:"
        echo "  sudo apt update"
        echo "  sudo apt install ros-humble-usb-cam"
    fi
else
    echo "✗ IFWATER camera NOT detected"
    echo ""
    echo "TROUBLESHOOTING STEPS:"
    echo ""
    echo "1. Check if camera is plugged in:"
    echo "   - Unplug the IFWATER camera"
    echo "   - Run: ls /dev/video* > before.txt"
    echo "   - Plug in the camera"
    echo "   - Run: ls /dev/video* > after.txt"
    echo "   - Compare: diff before.txt after.txt"
    echo "   - The NEW devices are your camera"
    echo ""
    echo "2. Try different USB ports (USB 3.0 preferred)"
    echo ""
    echo "3. Check USB connection:"
    echo "   lsusb"
    echo "   (Look for a new device when camera is plugged in)"
    echo ""
    echo "4. Check kernel messages:"
    echo "   dmesg | tail -50"
    echo "   (Look for USB connection messages)"
fi

echo ""
echo "========================================"