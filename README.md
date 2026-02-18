# lunar_rover_ws
## Connect to moonpie MiniPC
### MiniPC is named moonpie and has a static IP of 138.67.181.222
### run on laptop
ssh moonpie@138.67.181.222

### note all code is on the MiniPC and laptop. Both computers have to be on the same Wifi

## How to run:
cd ~/lunar_rover_ws
colcon build 
source install/setup.bash

## Python script with buttons to all needed ros nodes and rviz need to add RTAB-Map. Tests all on one computer (Not as usefull anymore)
python3 rover_launcher.py

## Run with mission control (laptop) and robot brain (MiniPC)
- MiniPC:
ssh moonpie@IP
bash mini_pc_launch.sh

- Laptop
bash laptop_ros_launch.sh

- 2nd Terminal (not tested)
pyhton3 laptop_control_gui.py


## Test arduino teleop code no ROS
cd /lunar_rover_ws/src/lunar_robot_hardware/lunar_robot_hardware/src
python3 teleop_no_ros.py

## Test Camera
cd DiagnosticAndTesting
bash test_camera_transforms.sh

## Teleop 
### Terminal 1
ros2 run lunar_robot_hardware arduino_motor_controller

### Terminal 2
ros2 run lunar_robot_hardware arduino_teleop

### Terminal 2 for point click navigation
ros2 launch lunar_robot_hardware arduino_navigation.launch.py

//////////////////////////////////////////////////////////////////////////
ToDo:
- need to update hardware conntrolling in ROS to use new hardware and an Arduino (Have a fix need to test arduino with and without ros)
- need to update telleop's (Added new teleop for arduino one with and without ros. Still need to add camera movment)
- need to fix RViz and RTAB map *(Work in progress currently looking into new solutions)
    - Is it possible to make and use the map all at once or do we need to make the map then use it?
- Add a better and more secure option for startup. (Current static IP and have two launch files a laptop and miniPC version, need to fix hardware control but have bad connection to camera over wifi)
    - I will not have access to a monitor for the miniPC on start so I need a headless startup.

//////////////////////////////////////////////////////////////////////////

## Integrate arduino code (Needs testing for both teleop and point click)


## Run Scripts for miniPC and laptop (Currently have two scrips, have only tested the camera's not the hardware yet)
Currnetly have a split launch that I am testing using laptop_ros_launch and mini_pc_launch.sh. I have the image's from the camera's however I am getting verry few frames per second. I also need to replace the current teleop with the arduino_teleop.py. I also have not gotten the laptop_control_gui.py to work. I want to GUI to have easy launch for all possible starts. just teleop, teleop and images, teleop and RGBD, teleop with point cloud, and point click navigation. No need for the other SLAM or RTAB in the GUI right now. 

## Polish everything for Regolith pit