#!/bin/bash
cd ~/lunar_rover_ws
source install/setup.bash

echo "=== MINI PC: Starting hardware nodes ==="

# TF + robot model
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p use_sim_time:=false &

sleep 1

# Motor controller
ros2 run lunar_robot_hardware motor_controller_node \
  --ros-args \
  -p fr_port:=/dev/ttyUSB0 \
  -p fl_port:=/dev/ttyUSB1 \
  -p br_port:=/dev/ttyUSB2 \
  -p bl_port:=/dev/ttyUSB3 &

# Front camera (D435)
ros2 launch realsense2_camera rs_launch.py \
  camera_name:=camera \
  enable_depth:=true \
  enable_color:=true \
  pointcloud.enable:=true &

# Rear stereo camera
python3 stereo_camera_publisher.py \
  --ros-args \
  -p device:=/dev/video32 \
  -p width:=1600 \
  -p height:=600 \
  -p fps:=30 &

wait
