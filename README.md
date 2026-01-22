# lunar_rover_ws
How to run:

cd ~/lunar_rover_ws
colcon build 
source install/setup.bash

// Python script with buttons to all needed ros nodes and rviz need to add RTAB-Map
python3 rover_launcher.py

// Test motors individualy 
python3 single_motor_test.py

// Test drive chain and actuators 
python3 test_drive.py

// Test Camera
ros2 launch realsense_camera rs_launch.py
ros2 run rviz2 rviz2

or

bash test_camera_transforms.sh

or 

bash rtabmap_with_navigation.sh

//////////////////////////////////////////////////////////////////////////
test_drive works. Have not tested OOP for driving and actuators.

Only have one working camera so have not tested navigation. The T265 camera is broken. 
Need to replace the T265 camera with the new camera IFWATER 3D Stereo USB Camera 1080P

When runnig test_camera_transforms.sh I can get point cloud but I cannot get the point cloud with the other launch from rover_launcher.py. I can get the image but thats it. 
RTAB-Map currently dose not work, bash rtabmap_with_navigation.sh terminates on start. When manualy running the rtabmap terminals 3 and 4 give warrnins and I cannot get the RTAB-Map window to launch. 

//////////////////////////////////////////////////////////////////////////
cd ~/lunar_rover_ws
colcon build
source install/setup.bash


// Hardware
ros2 run lunar_robot_hardware motor_controller_node

// Camera 
ros2 launch realsense2_camera rs_launch.py

ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame

// Navigation
ros2 run lunar_robot_autonomous unified_navigator

or 

ros2 run lunar_robot_autonomous rtabmap_multi_waypoint

// Teleop 
python3 teleop_keyboard.py

or 

python3 controller_teleop.py