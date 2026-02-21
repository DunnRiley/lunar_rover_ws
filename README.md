# lunar_rover_ws
## Connect to moonpie MiniPC
### MiniPC is named moonpie and has a static IP of 192.168.0.102
### run on laptop
ssh moonpie@192.168.0.102

### note all code is on the MiniPC and laptop. Both computers have to be on the same Wifi

## How to run:
cd ~/lunar_rover_ws
colcon build 
source install/setup.bash

## Run from mission control (laptop)
bash full_launch_laptop.sh --start-minipc

## SLAM with RTAB-mapping (Not Implemented)
- MiniPC:
bash slam_minipc.sh mapping

- Laptop:
bash slam_laptop.sh

######################################### OLD #########################################

## Python script with buttons to all needed ros nodes and rviz need to add RTAB-Map. Tests all on one computer (Not as usefull anymore)
python3 rover_launcher.py

## Split launch for cameras's
- MiniPC:
ssh moonpie@IP
bash mini_pc_launch.sh

- 2nd Terminal MiniPC:
pyhton3 optimized_image_pipeline.py

- Laptop:
bash laptop_ros_launch.sh

## Test Camera
cd DiagnosticAndTesting
bash test_camera_transforms.sh

## Test arduino teleop code no ROS (WORKING)
cd /lunar_rover_ws/ArduinoNoROS
python3 teleop_no_ros.py

## Teleop 
### Terminal 1 Connect to Arduino
ros2 run lunar_robot_hardware arduino_motor_controller

### Terminal 2 Keyboard telep
ros2 run lunar_robot_hardware arduino_teleop

## Terminal 2 Controler Start
ros2 run joy joy_node

## Terminal 3 Controler Teleop
ros2 run lunar_robot_hardware controller_teleop

### Terminal 2 for point click navigation
ros2 launch lunar_robot_hardware arduino_navigation.launch.py


