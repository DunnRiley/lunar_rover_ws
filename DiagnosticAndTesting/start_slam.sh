#!/bin/bash

echo "========================================="
echo "  SLAM Navigation Startup"
echo "========================================="
echo ""
echo "Starting SLAM system..."
cd ~/lunar_rover_ws
source install/setup.bash

# Launch SLAM in background
ros2 launch lunar_robot_hardware slam_navigation.launch.py &
SLAM_PID=$!

# Wait for system to start
sleep 5

echo ""
echo "========================================="
echo "  System Ready!"
echo "========================================="
echo ""
echo "To build the map:"
echo "  1. Open a new terminal"
echo "  2. Run: cd ~/lunar_rover_ws && python3 teleop_keyboard.py"
echo "  3. Drive around slowly for 10-20 seconds"
echo ""
echo "Once map appears in RViz:"
echo "  - Click 'Publish Point' to add waypoints"
echo "  - Click '2D Goal Pose' to start navigation"
echo "  - Map will update as you drive!"
echo ""
echo "Press Ctrl+C to stop everything"
echo "========================================="

# Wait for user to stop
wait $SLAM_PID