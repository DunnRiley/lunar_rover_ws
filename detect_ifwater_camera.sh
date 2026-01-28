#!/bin/bash
# IFWATER Stereo Camera Detection Script
# Helps identify which /dev/video* devices are the left and right cameras

echo "========================================"
echo "  IFWATER 3D Stereo Camera Detector    "
echo "========================================"
echo ""

echo "1. Looking for video devices..."
if ls /dev/video* 2>/dev/null; then
    echo ""
else
    echo "  ✗ No video devices found!"
    echo ""
    echo "  - Make sure the IFWATER camera is plugged in"
    echo "  - Try a different USB port"
    echo "  - Check if USB port has power"
    exit 1
fi

echo ""
echo "2. Checking video device details..."
echo ""

for device in /dev/video*; do
    echo "Device: $device"
    
    # Get device name and capabilities
    v4l2-ctl --device=$device --info 2>/dev/null | grep -E "Card type|Capabilities" || echo "  (No info available)"
    
    # Check if device supports video capture
    if v4l2-ctl --device=$device --list-formats 2>/dev/null | grep -q "Video Capture"; then
        echo "  ✓ Video Capture: YES"
        
        # List available formats
        echo "  Available formats:"
        v4l2-ctl --device=$device --list-formats-ext 2>/dev/null | grep -E "^\[|Size:|Interval" | head -20
    else
        echo "  ✗ Video Capture: NO (metadata device, skip this one)"
    fi
    
    echo ""
done

echo "========================================"
echo "  IFWATER Camera Setup Guide           "
echo "========================================"
echo ""
echo "The IFWATER 3D Stereo camera has TWO lenses and appears"
echo "as TWO separate video devices in Linux."
echo ""
echo "Typically:"
echo "  Left camera:  /dev/video2"
echo "  Right camera: /dev/video4"
echo ""
echo "But on your system, check the output above."
echo "Look for devices that support 'Video Capture' and"
echo "have 1920x1080 resolution available."
echo ""
echo "Testing tip:"
echo "  Test each device with:"
echo "  ffplay /dev/video2"
echo "  ffplay /dev/video4"
echo ""
echo "If ffplay is not installed:"
echo "  sudo apt install ffmpeg"
echo ""
echo "========================================"
echo "  Quick Test Commands                  "
echo "========================================"
echo ""
echo "Test left camera (usually /dev/video2):"
echo "  ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video2 -p image_width:=1920 -p image_height:=1080"
echo ""
echo "Test right camera (usually /dev/video4):"
echo "  ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video4 -p image_width:=1920 -p image_height:=1080"
echo ""
echo "View in RViz:"
echo "  Add 'Image' display"
echo "  Topic: /image_raw"
echo ""