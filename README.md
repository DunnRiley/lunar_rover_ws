# lunar_rover_ws
How to run:
Python script with buttons to all needed ros nodes and rviz
cd ~/lunar_rover_ws
python3 rover_launcher.py

Test motors:
individual motor
cd ~/lunar_rover_ws
python3 single_motor_test.py

test drive chain and actuators 
cd ~/lunar_rover_ws
python3 test_drive.py

Test Camera
ros2 launch realsense_camera rs_launch.py
ros2 run rviz2 rviz2

bash test_camera_transforms.sh

//////////////////////////////////////////////////////////////////////////

test_drive works
have not tested OOP for driving and actuators

Only have one working camera so have not tested navigation. The T265 camera is broken. 
Need to replace the T265 camera with the new camera IFWATER 3D Stereo USB Camera 1080P

Currently having issues with the D435 camera in the ros rviz launch.
When runnig test_camera_transforms.sh I can get point cloud but I cannot get the point cloud with the other launch from rover_launcher.py, I can get the image but thats it. 

colcon build
source install/setup.bash

ros2 run lunar_robot_hardware motor_controller_node

ros2 launch realsense2_camera rs_launch.py

ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame

ros2 run lunar_robot_autonomous unified_navigator

python3 teleop_keyboard.py