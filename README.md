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

## Python script with buttons to all needed ros nodes and rviz need to add RTAB-Map. Tests all on one computer (Not as usefull anymore)
python3 rover_launcher.py

## Test Camera
cd DiagnosticAndTesting
bash test_camera_transforms.sh

## Run with mission control (laptop) and robot brain (MiniPC)
- MiniPC:
ssh moonpie@IP
bash mini_pc_launch.sh

- 2nd Terminal MiniPC:
pyhton3 optimized_image_pipeline.py

- Laptop:
bash laptop_ros_launch.sh


## Test arduino teleop code no ROS
cd /lunar_rover_ws/src/lunar_robot_hardware/lunar_robot_hardware/src
python3 teleop_no_ros.py

## Teleop (Not integrated into split computer's)
### Terminal 1
ros2 run lunar_robot_hardware arduino_motor_controller

### Terminal 2
ros2 run lunar_robot_hardware arduino_teleop

### Terminal 2 for point click navigation
ros2 launch lunar_robot_hardware arduino_navigation.launch.py

## SLAM with RTAB-mapping (Not Implemented)
- MiniPC:
bash slam_minipc.sh mapping

- Laptop:
bash slam_laptop.sh


//////////////////////////////////////////////////////////////////////////
ToDo:
- need to update hardware conntrolling in ROS to use new hardware and an Arduino (Have a fix need to test arduino with and without ros)
- need to update telleop's (Still need to add camera movment. Also need to add new teleop to either launch files)
- need to fix RViz and RTAB map *(Work in progress currently looking into new solutions)
- Is it possible to make and use the map all at once or do we need to make the map then use it?
- Add a better and more secure option for startup. (Current static IP and have two launch files a laptop and miniPC version, need to fix hardware control)

//////////////////////////////////////////////////////////////////////////

## Integrate arduino code (Needs testing for both teleop and point click)


## Run Scripts for miniPC and laptop (Currently have two scrips, have only tested the camera's not the hardware yet)
Currnetly have a split launch that I am testing using laptop_ros_launch and mini_pc_launch.sh. I also need to replace the current teleop with the arduino_teleop.py. I also have not gotten the laptop_control_gui.py to work. I want a GUI to have easy launch for all possible starts. just teleop, teleop and images, teleop and RGBD, teleop with point cloud, and point click navigation. No need for the other SLAM or RTAB in the GUI right now.

## Point cloud and SLAM
I am working on a project with a rover that has cameras and multiple ros nodes. Can you help me work on my SLAM and RTab-mapping. My current implementation uses two computers, one mission control that has terminals and a robot brain that dose not have accese to a monitor (headless). I am connectiong over ssh to the robot brain and running two scripts, slam_laptop.sh, slam_minipc.sh. I do not have a working maping system. I am getting an error from the miniPC launch, this error is saved in Error.txt, and have not gotten to the point of testing RViz with the laptop launch


## Polish everything for Regolith pit by 02