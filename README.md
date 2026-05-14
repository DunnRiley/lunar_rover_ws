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

## On Mini PC for automation
bash ~/lunar_rover_ws/run_mission.sh --dry-run mission.yaml
bash ~/lunar_rover_ws/run_mission.sh

## Speaker
# Note A4
speaker-test -t sine -f 440

# Music
mplayer chiefkeefsosa.mp3

## Record Cam
rs-record -f depth_only.bag -t 5

rs-convert -i depth_only.bag -p depth_frame


#### OLD ####
## On mini pc
# Terminal 1 - Arduino motor bridge
  ros2 run lunar_robot_hardware arduino_motor_controller

# Terminal 2 - Nav processor (A* + obstacle avoidance)
  python3 nav_depth_processor.py

# Terminal 3 - Command mux (joystick vs autonomous)
    python3 nav_cmd_mux.py

# Terminal 4 - (optional) depth odometry
python3 nav_depth_odom.py

# Laptop 
  python3 nav_control_panel.py


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