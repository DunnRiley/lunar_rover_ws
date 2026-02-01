## lunar_rover_ws
# How to run:

cd ~/lunar_rover_ws
colcon build 
source install/setup.bash

# Python script with buttons to all needed ros nodes and rviz need to add RTAB-Map
python3 rover_launcher.py

# Test motors individualy 
cd DiagnosticAndTesting
python3 single_motor_test.py

# Test drive chain and actuators 
cd DiagnosticAndTesting
python3 test_drive.py

# Test Camera
ros2 launch realsense_camera rs_launch.py
ros2 run rviz2 rviz2

# or
cd DiagnosticAndTesting
bash test_camera_transforms.sh

# or 
// not tested
bash rtabmap_with_navigation.sh

//////////////////////////////////////////////////////////////////////////
test_drive works. Have not tested OOP for driving and actuators. In the prosses of adding arduino and encoders

When runnig test_camera_transforms.sh and rover_launcher.py (Launch Cameras Test) I can get point cloud. 
RTAB-Map and SLAM currently do not work. 

//////////////////////////////////////////////////////////////////////////
cd ~/lunar_rover_ws
colcon build
source install/setup.bash

# Hardware
ros2 run lunar_robot_hardware motor_controller_node

# Camera 
# Step 1: Fix USB stability (run once per boot)
cd ~/lunar_rover_ws
chmod +x fix_camera_usb.sh
sudo ./fix_camera_usb.sh

# Step 2 Launch 
ros2 launch realsense2_camera rs_launch.py

ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame

# Navigation: point and click with slam
ros2 launch lunar_robot_hardware simple_slam_nav.launch.py

# Teleop 
python3 teleop_keyboard.py

# or 

python3 controller_teleop.py