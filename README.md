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
bash laptop_ros_launch

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

