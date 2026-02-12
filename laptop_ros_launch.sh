#!/bin/bash

echo "=== LAPTOP ROS2 CONTROL SYSTEM ==="

cd ~/lunar_rover_ws
source install/setup.bash

# Safety delay to ensure mini PC nodes are up
sleep 2

echo "Starting SLAM / Navigation..."
ros2 launch lunar_robot_hardware slam_navigation.launch.py &

sleep 2

echo "Starting RViz..."
ros2 run rviz2 rviz2 -d ~/lunar_rover_ws/hardware_navigation.rviz &

sleep 2

echo "Starting Teleop..."
python3 teleop_keyboard.py &

wait
