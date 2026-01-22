#!/bin/bash
# RTAB-Map Multi-Waypoint Navigation - FIXED timing issues
# Waits for camera to be fully ready before starting RTAB-Map

echo "   RTAB-Map Multi-Waypoint Navigation System           "
echo ""

# Check if parameter files exist
if [ ! -f ~/lunar_rover_ws/rtabmap_odom_params.yaml ]; then
    echo "ERROR: Missing rtabmap_odom_params.yaml"
    echo "Run: cd ~/lunar_rover_ws && cat > rtabmap_odom_params.yaml"
    exit 1
fi

if [ ! -f ~/lunar_rover_ws/rtabmap_slam_params.yaml ]; then
    echo "ERROR: Missing rtabmap_slam_params.yaml"
    exit 1
fi

echo "Parameter files found"
echo ""
echo "Starting system... (this takes ~30 seconds)"
echo "Press Ctrl+C in THIS terminal to stop everything"
echo ""
sleep 2

# Kill any existing ROS nodes
echo "Cleaning up old processes..."
pkill -f ros2
pkill -f rviz2
pkill -f rtabmap
pkill -f realsense
sleep 2

# Source workspace
cd ~/lunar_rover_ws
source install/setup.bash

echo ""
echo "1/7: Starting robot state publisher..."

ros2 run robot_state_publisher robot_state_publisher \
    --ros-args \
    -p robot_description:="<?xml version='1.0'?>
<robot name='lunar_rover'>
  <link name='base_link'>
    <visual>
      <geometry>
        <box size='0.5 0.3 0.2'/>
      </geometry>
    </visual>
  </link>
  <link name='camera_link'/>
  <joint name='base_to_camera' type='fixed'>
    <parent link='base_link'/>
    <child link='camera_link'/>
    <origin xyz='0.15 0 0.2' rpy='0 0 0'/>
  </joint>
</robot>" \
    -p use_sim_time:=false > /dev/null 2>&1 &

RSP_PID=$!
sleep 2
echo "Robot state publisher started (PID: $RSP_PID)"

echo ""
echo "2/7: Publishing TF transforms..."

ros2 run tf2_ros static_transform_publisher \
    0 0 0 \
    -1.5707963267948966 0 -1.5707963267948966 \
    camera_link camera_depth_optical_frame > /dev/null 2>&1 &
TF1_PID=$!

ros2 run tf2_ros static_transform_publisher \
    0 0 0 \
    -1.5707963267948966 0 -1.5707963267948966 \
    camera_link camera_color_optical_frame > /dev/null 2>&1 &
TF2_PID=$!

ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 \
    odom base_link > /dev/null 2>&1 &
TF3_PID=$!

sleep 1
echo "TF transforms published"

echo ""
echo "3/7: Starting D435 camera (this takes ~10 seconds)..."

ros2 launch realsense2_camera rs_launch.py \
    camera_name:=camera \
    camera_namespace:=camera \
    enable_depth:=true \
    enable_color:=true \
    pointcloud.enable:=true \
    align_depth.enable:=true \
    depth_module.profile:=640x480x30 \
    rgb_camera.profile:=640x480x30 > /tmp/camera.log 2>&1 &

CAM_PID=$!

# Wait for camera to actually start publishing
echo "   Waiting for camera initialization..."
sleep 8

# Check if camera topics exist
echo "   Checking camera topics..."
CAMERA_READY=0
for i in {1..10}; do
    if ros2 topic list 2>/dev/null | grep -q "/camera/camera/color/image_raw"; then
        CAMERA_READY=1
        break
    fi
    echo "   Attempt $i/10..."
    sleep 1
done

if [ $CAMERA_READY -eq 1 ]; then
    echo "Camera topics detected"
    
    # Verify images are actually being published
    echo "   Verifying image stream..."
    timeout 3 ros2 topic hz /camera/camera/color/image_raw > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "Camera is publishing images"
    else
        echo "Camera topics exist but no data yet (will retry)"
    fi
else
    echo "Camera topics not detected (check /tmp/camera.log)"
    echo "   Continuing anyway..."
fi

echo ""
echo "4/7: Starting RGB-D Odometry..."

ros2 run rtabmap_odom rgbd_odometry \
    --ros-args \
    --params-file ~/lunar_rover_ws/rtabmap_odom_params.yaml \
    --remap rgb/image:=/camera/camera/color/image_raw \
    --remap rgb/camera_info:=/camera/camera/color/camera_info \
    --remap depth/image:=/camera/camera/aligned_depth_to_color/image_raw > /tmp/odom.log 2>&1 &

ODOM_PID=$!
sleep 3

# Check if odometry started
if ps -p $ODOM_PID > /dev/null; then
    echo "RGB-D Odometry started (PID: $ODOM_PID)"
else
    echo "✗ RGB-D Odometry failed to start (check /tmp/odom.log)"
fi

echo ""
echo "5/7: Starting RTAB-Map SLAM..."

ros2 run rtabmap_slam rtabmap \
    --ros-args \
    --params-file ~/lunar_rover_ws/rtabmap_slam_params.yaml \
    -r delete_db_on_start:=true \
    --remap rgb/image:=/camera/camera/color/image_raw \
    --remap rgb/camera_info:=/camera/camera/color/camera_info \
    --remap depth/image:=/camera/camera/aligned_depth_to_color/image_raw > /tmp/rtabmap.log 2>&1 &

RTABMAP_PID=$!
sleep 3

# Check if RTAB-Map started
if ps -p $RTABMAP_PID > /dev/null; then
    echo "RTAB-Map started (PID: $RTABMAP_PID)"
else
    echo "✗ RTAB-Map failed to start (check /tmp/rtabmap.log)"
fi

echo ""
echo "6/7: Checking system status..."

sleep 2

echo ""
echo "Camera topics:"
ros2 topic list 2>/dev/null | grep "/camera/camera" | head -5

echo ""
echo "RTAB-Map topics:"
ros2 topic list 2>/dev/null | grep rtabmap

echo ""
echo "Checking data flow (this may take a moment)..."
timeout 3 ros2 topic hz /odom 2>/dev/null && echo "Odometry publishing" || echo "No odometry data yet"
timeout 3 ros2 topic hz /rtabmap/grid_map 2>/dev/null && echo "RTAB-Map grid publishing" || echo "No grid map yet (will appear after camera movement)"

echo ""
echo "7/7: Starting Multi-Waypoint Navigator..."

ros2 run lunar_robot_autonomous multi_waypoint_navigator \
    --ros-args \
    -p goal_tolerance:=0.3 \
    -p forward_speed:=0.25 \
    -p turn_speed:=0.4 > /tmp/navigator.log 2>&1 &

NAV_PID=$!
sleep 2

if ps -p $NAV_PID > /dev/null; then
    echo "Multi-waypoint navigator started (PID: $NAV_PID)"
else
    echo "✗ Navigator failed (check /tmp/navigator.log)"
fi

echo ""
echo "Starting RViz..."
echo ""

# Launch RViz
if [ -f ~/lunar_rover_ws/rtabmap_navigation.rviz ]; then
    ros2 run rviz2 rviz2 -d ~/lunar_rover_ws/rtabmap_navigation.rviz > /dev/null 2>&1 &
else
    ros2 run rviz2 rviz2 > /dev/null 2>&1 &
fi

RVIZ_PID=$!
sleep 2

echo ""
echo "              SYSTEM READY                              "
echo "                                                        "
echo "  IF YOU SEE WARNINGS ABOUT 'Did not receive data':    "
echo "  - This is NORMAL at startup                           "
echo "  - Check /tmp/camera.log if camera isn't working       "
echo "  - Move/rotate camera to start building map            "
echo "                                                        "
echo "  IN RViz:                                              "
echo "  1. Set Fixed Frame to 'map'                           "
echo "  2. Enable 'RTAB-Map Cloud (Persistent)'               "
echo "  3. Enable 'Occupancy Grid'                            "
echo "  4. Rotate camera 360° to build map                    "
echo "  5. Click 'Publish Point' to add waypoints             "
echo "  6. Click '2D Nav Goal' to start navigation            "
echo "                                                        "
echo "  TROUBLESHOOTING:                                      "
echo "  - Check logs: cat /tmp/camera.log                     "
echo "  - Check logs: cat /tmp/odom.log                       "
echo "  - Check logs: cat /tmp/rtabmap.log                    "
echo "                                                        "
echo "  Press Ctrl+C to stop all nodes                        "
echo "                                                        "
echo ""

# Save PIDs for cleanup
echo "Process IDs:"
echo "  Robot State Publisher: $RSP_PID"
echo "  Camera: $CAM_PID"
echo "  Odometry: $ODOM_PID"
echo "  RTAB-Map: $RTABMAP_PID"
echo "  Navigator: $NAV_PID"
echo "  RViz: $RVIZ_PID"
echo ""

# Trap handler for clean shutdown
cleanup() {
    echo ""
    echo "Stopping all nodes..."
    kill $RSP_PID $TF1_PID $TF2_PID $TF3_PID $CAM_PID $ODOM_PID $RTABMAP_PID $NAV_PID $RVIZ_PID 2>/dev/null
    sleep 2
    pkill -f ros2
    pkill -f rviz2
    pkill -f rtabmap
    pkill -f realsense
    echo "Stopped"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Keep script running
echo "System running. Monitoring processes..."
echo "(Press Ctrl+C to stop)"
echo ""

# Monitor critical processes
while true; do
    sleep 5
    
    # Check if critical processes are still running
    if ! ps -p $CAM_PID > /dev/null 2>&1; then
        echo "Camera process died! Check /tmp/camera.log"
    fi
    
    if ! ps -p $ODOM_PID > /dev/null 2>&1; then
        echo "Odometry process died! Check /tmp/odom.log"
    fi
    
    if ! ps -p $RTABMAP_PID > /dev/null 2>&1; then
        echo "RTAB-Map process died! Check /tmp/rtabmap.log"
    fi
done