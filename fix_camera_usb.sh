#!/bin/bash
# Fix USB stability issues for RealSense D435 camera
# Run this BEFORE starting RTAB-Map

echo "=================================="
echo "  USB CAMERA STABILITY FIXER"
echo "=================================="
echo ""

# 1. Disable USB autosuspend (main cause of disconnects)
echo "1. Disabling USB autosuspend for RealSense devices..."
sudo sh -c 'echo -1 > /sys/module/usbcore/parameters/autosuspend'

# Find RealSense USB device
REALSENSE_DEVICE=$(lsusb | grep "Intel" | grep -oP 'Bus \d+ Device \d+' | head -1)

if [ -n "$REALSENSE_DEVICE" ]; then
    BUS=$(echo $REALSENSE_DEVICE | grep -oP 'Bus \K\d+')
    DEV=$(echo $REALSENSE_DEVICE | grep -oP 'Device \K\d+')
    echo "   Found RealSense on Bus $BUS, Device $DEV"
    
    # Disable autosuspend for this specific device
    USB_PATH=$(find /sys/bus/usb/devices/ -name "power" | grep "$BUS-" | head -1 | xargs dirname)
    if [ -n "$USB_PATH" ]; then
        echo "   Disabling autosuspend: $USB_PATH"
        sudo sh -c "echo 'on' > $USB_PATH/power/control"
        sudo sh -c "echo '0' > $USB_PATH/power/autosuspend"
        sudo sh -c "echo '0' > $USB_PATH/power/autosuspend_delay_ms"
    fi
else
    echo "   Warning: RealSense device not found in lsusb"
fi

# 2. Increase USB buffer size
echo ""
echo "2. Increasing USB buffer size..."
sudo sh -c 'echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb'

# 3. Set RealSense udev rules if not already set
echo ""
echo "3. Checking udev rules..."
if [ ! -f /etc/udev/rules.d/99-realsense-libusb.rules ]; then
    echo "   Installing RealSense udev rules..."
    sudo wget -O /etc/udev/rules.d/99-realsense-libusb.rules https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules 2>/dev/null
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "   Udev rules installed"
else
    echo "   Udev rules already present"
fi

# 4. Check USB connection quality
echo ""
echo "4. USB Connection Status:"
echo "   ----------------------"
lsusb -t | grep -A 2 Intel

# 5. Recommendations
echo ""
echo "=================================="
echo "  RECOMMENDATIONS"
echo "=================================="
echo ""
echo "✓ USB autosuspend disabled"
echo "✓ USB buffers increased"
echo ""
echo "Additional tips:"
echo "  - Use USB 3.0 port (blue port)"
echo "  - Use short, high-quality USB cable"
echo "  - Avoid USB hubs if possible"
echo "  - Check cable connection is tight"
echo "  - Consider powered USB hub if needed"
echo ""
echo "If camera still disconnects:"
echo "  - Try different USB port"
echo "  - Check dmesg for errors: dmesg | grep -i usb"
echo "  - Monitor USB power: watch -n 1 'lsusb -v | grep -A 5 Intel'"
echo ""
echo "=================================="
echo "  USB fixes applied! Now run:"
echo "  ros2 launch lunar_robot_hardware rtabmap_stable.launch.py"
echo "=================================="