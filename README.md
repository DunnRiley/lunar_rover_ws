# lunar_rover_ws
## Connect to cheese MiniPC
### MiniPC is named cheese and has a static IP of 192.168.0.102
### run on laptop
ssh cheese@192.168.0.102

### note all code is on the MiniPC and laptop. Both computers have to be on the same Wifi

## How to run:
cd ~/lunar_rover_ws
colcon build 
source install/setup.bash

## Run from mission control (laptop)
bash full_launch_laptop.sh --start-minipc

## Run On Mini PC after RViz is launched for stereo camera
bash ~/lunar_rover_ws/restart_rear_camera.sh

## SLAM with RTAB-mapping (Not Implemented)
python3 slam_launch.py
bash slam_laptop.sh

## Kill topics
pkill -f joy_node
pkill -f arduino_teleop_controller

## Commands for Arduino
All Commands shall be sent in the Following Format 
N/A values means it doesn’t matter. 

Start | Device | Speed | Direction | END | Description
0xAA | 0x05 | Int 0-255 | Int 0 or 1 | 0x55 | Controls left motors 
0xAA | 0x06 | Int 0-255 | Int 0 or 1 | 0x55 | Controls right motors 
0xAA | 0x08 | Int 0-255 | Int 0 or 1 | 0x55 | Controls Actuators 
0xAA | 0x11 | Int 0-255 | N/A | 0x55 | Controls Servo, 90 is stop, 45 CC, 135 CW 
0xAA | 0xA7 | N/A | N/A | 0x55 | Sets Actuator to Dig 
0xAA | 0xA9 | N/A | N/A | 0x55 | Sets Actuator to Drive  
0xAA | 0xB3 | N/A | N/A | 0x55 | Sets Actuator to Dump 
0xAA | 0xCA | N/A | N/A | 0x55 | Set Motor to high level, encoder counts to 0, ( Calibrate Bucket) 
0xAA | 0xB4 | N/A | N/A | 0x55 | Stops all moving parts 
0xAA | 0xD1 | N/A | N/A | 0x55 | Request  IMU Data NOW 


## Signals back from IMU/encoders
Telemetry 
Start: 0xAA 
ax: 4 byte float 
ay: 4 byte float 
az: 4 byte float 
gx: 4 byte float 
gy: 4 byte float 
gz: 4 byte float 
ENC Start: 0xA5 
ENC: 2 byte integer 16 bits, min 0 max 65535 
END: 0x55 

encoders are actuators only.
a is axelaration, g is gyro.

Unpack gyro/accelaration data
imprt struct
ang_cdeg = struct.unpacj('<h' data)[0]
angle_deg = ang_cdeg / 1000.0


# Important dependencies
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-navigation2 ros-jazzy-nav2-route

############################ Needed Dependencies for SLAM ############################
sudo apt update
sudo apt install -y \
  ros-jazzy-rtabmap \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-rtabmap-slam \
  ros-jazzy-rtabmap-odom \
  ros-jazzy-rtabmap-viz \
  ros-jazzy-rtabmap-msgs \
  ros-jazzy-rtabmap-examples

######################################### OLD #########################################

## Python script with buttons to all needed ros nodes and rviz need to add RTAB-Map. Tests all on one computer (Not as usefull anymore)
python3 rover_launcher.py

## Split launch for cameras's
- MiniPC:
ssh cheese@IP
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


//////////////////////////////////////////////////////////////////////////
ToDo:
- Get Point click navigation WORKING by 2/17
  - add object detection
  - add objects to rviz 3d map
  - genorate path from rover to a clicked point
  - use IMU data to stay on the path
- need to fix RViz and RTAB map *(Work in progress currently looking into new solutions)
    -  Is it possible to make and use the map all at once or do we need to make the map then use it?
- Add a better and more secure option for startup. (Current static IP and one launch file)

//////////////////////////////////////////////////////////////////////////

## Point click
I am working with streaming a D435 camera from a mini pc to a laptop. I am using RViz to stream the image to my laptop but I would also like to use nav2 and a point click navigation to avoid objects. I attempted to make a GUI to allow me to interact wiht the Depth image but I cant get object detection to work and if I manualy select an object on the GUI i can keep track of the object as long as it stays in frame. However none of these points show up in Rviz. The GUI is nice but i am looking more for a point click navigation that detects obsticasl and can makes a path to avoid the obsticals. most of the obsticals will be about 30 to 40 cm rocks. While streaming I dont stream the point cloud becasue it lags to much. The camera is about 75 cm above the ground at a 25 degree angle. I can't see what dirrectly infront of the rover so id like to make the path so as I approch an object I stay far enough away. I am also moving in regolith so mobility is not the best and I dont have encoder data. Can you help me add object detection and keep the manuale selection if it fails, I would like the obsticals to appear in the 3d rviz window so a path can the be drawn and nav2 can be used. 

Can you help me get a basic point click navigation for my rover. I was attempting to have a new GUI appear so it can do object detections and path planning that is sent to RViz to then use nav2 but ht dose not work. Can you help me get a basic autonomas working? I would like the basic point click so that when a point in front of the rover is clicked the rover will drive to that point. I am struggeling with the object detection becasue the camera is about 70 cm off the ground and is at a 25 degree angle so the veiw shows a gradeaint going ferther away leading to challenges with object detection. for now I was trying to implement the ability to manualy click on objects and seting that as an obstical but currently non of this data is getting to RViz. Can you help me simplify it for a test? I would like to select a point in veiw that could be a few meters away and drie untill I reach that point. I am now able to get IMU data aswell.


## Teleop

## Point cloud and SLAM
Ferther down the line
