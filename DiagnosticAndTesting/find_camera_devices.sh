#!/bin/bash
# Advanced IFWATER Camera Device Finder
# Finds the actual capture devices (not metadata devices)

echo "========================================"
echo "  IFWATER Camera Device Finder         "
echo "========================================"
echo ""

echo "Step 1: Finding all video devices..."
echo ""

# Kill any processes using cameras
echo "Killing any existing camera processes..."
pkill -9 ffplay 2>/dev/null
pkill -9 -f simple_camera_publisher 2>/dev/null
sleep 1

echo ""
echo "Step 2: Testing each device for capture capability..."
echo ""

CAPTURE_DEVICES=()

for device in /dev/video*; do
    device_num=$(basename $device | sed 's/video//')
    
    # Check if it's a capture device
    caps=$(v4l2-ctl --device=$device --all 2>/dev/null | grep "Video Capture")
    
    if [ -n "$caps" ]; then
        # Try to get a format
        format=$(v4l2-ctl --device=$device --list-formats 2>/dev/null | grep -A1 "\[0\]" | tail -1)
        
        if [ -n "$format" ]; then
            # Try to capture a single frame to verify it really works
            timeout 2 ffmpeg -f v4l2 -video_size 640x480 -i $device -frames:v 1 /tmp/test_$device_num.jpg -y >/dev/null 2>&1
            
            if [ $? -eq 0 ]; then
                echo "✓ $device - WORKING CAPTURE DEVICE"
                
                # Get device info
                card=$(v4l2-ctl --device=$device --info 2>/dev/null | grep "Card type" | cut -d: -f2 | xargs)
                
                # Get supported resolutions
                resolutions=$(v4l2-ctl --device=$device --list-formats-ext 2>/dev/null | grep "Size:" | head -3)
                
                echo "    Card: $card"
                echo "    Test image saved: /tmp/test_$device_num.jpg"
                echo "    Supported sizes:"
                echo "$resolutions" | while read line; do
                    echo "      $line"
                done
                
                CAPTURE_DEVICES+=($device)
                echo ""
            else
                echo "○ $device - Exists but cannot capture"
                echo "    (likely a metadata device)"
                echo ""
            fi
        fi
    fi
done

echo "========================================"
echo "  RESULTS                               "
echo "========================================"
echo ""

if [ ${#CAPTURE_DEVICES[@]} -eq 0 ]; then
    echo "✗ No working capture devices found!"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Make sure camera is plugged in"
    echo "  2. Try unplugging and replugging the camera"
    echo "  3. Check USB connection: lsusb"
    exit 1
fi

echo "Found ${#CAPTURE_DEVICES[@]} working capture device(s):"
echo ""

for i in "${!CAPTURE_DEVICES[@]}"; do
    device="${CAPTURE_DEVICES[$i]}"
    device_num=$(basename $device | sed 's/video//')
    
    echo "Device $((i+1)): $device"
    
    # Show preview
    if [ -f "/tmp/test_$device_num.jpg" ]; then
        echo "  Preview saved at: /tmp/test_$device_num.jpg"
        
        # Try to identify which is left/right
        if [ $i -eq 0 ]; then
            echo "  → This is likely your LEFT camera"
        elif [ $i -eq 1 ]; then
            echo "  → This is likely your RIGHT camera"
        fi
    fi
    echo ""
done

echo "========================================"
echo "  RECOMMENDED CONFIGURATION             "
echo "========================================"
echo ""

if [ ${#CAPTURE_DEVICES[@]} -ge 2 ]; then
    LEFT_DEVICE="${CAPTURE_DEVICES[0]}"
    RIGHT_DEVICE="${CAPTURE_DEVICES[1]}"
    
    echo "For stereo camera setup:"
    echo "  LEFT_DEVICE:  $LEFT_DEVICE"
    echo "  RIGHT_DEVICE: $RIGHT_DEVICE"
    echo ""
    
    # Save configuration
    cat > ~/lunar_rover_ws/camera_config.txt << EOF
# IFWATER Stereo Camera Configuration
# Generated: $(date)
LEFT_DEVICE=$LEFT_DEVICE
RIGHT_DEVICE=$RIGHT_DEVICE
EOF
    
    echo "Configuration saved to: ~/lunar_rover_ws/camera_config.txt"
    echo ""
    
    echo "Test individual cameras with:"
    echo "  ffplay $LEFT_DEVICE"
    echo "  ffplay $RIGHT_DEVICE"
    echo ""
    
    echo "Test with ROS2 publisher:"
    echo "  python3 simple_camera_publisher.py --ros-args -p left_device:=$LEFT_DEVICE -p right_device:=$RIGHT_DEVICE"
    
elif [ ${#CAPTURE_DEVICES[@]} -eq 1 ]; then
    echo "Only 1 capture device found: ${CAPTURE_DEVICES[0]}"
    echo ""
    echo "This might be:"
    echo "  - A single camera"
    echo "  - One lens of your stereo camera"
    echo "  - The camera is not fully detected"
    echo ""
    echo "Try:"
    echo "  1. Unplug and replug the camera"
    echo "  2. Try a different USB port"
    echo "  3. Run: lsusb | grep -i camera"
fi

echo ""
echo "========================================"
echo "  VISUAL PREVIEW                        "
echo "========================================"
echo ""

if command -v eog >/dev/null 2>&1; then
    echo "Opening captured images..."
    for i in "${!CAPTURE_DEVICES[@]}"; do
        device="${CAPTURE_DEVICES[$i]}"
        device_num=$(basename $device | sed 's/video//')
        if [ -f "/tmp/test_$device_num.jpg" ]; then
            eog "/tmp/test_$device_num.jpg" 2>/dev/null &
        fi
    done
    echo "Check the opened images to verify camera views"
else
    echo "View captured test images with:"
    for i in "${!CAPTURE_DEVICES[@]}"; do
        device="${CAPTURE_DEVICES[$i]}"
        device_num=$(basename $device | sed 's/video//')
        if [ -f "/tmp/test_$device_num.jpg" ]; then
            echo "  /tmp/test_$device_num.jpg"
        fi
    done
fi

echo ""