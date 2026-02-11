## lunar_rover_ws
# Connect to moonpie MiniPC
// MiniPC is named moonpie and has a static IP of 138.67.181.222
// run on laptop
ssh moonpie@138.67.181.222

// note all code is on the MiniPC and both computers have to be on the same Wifi

# How to run:
cd ~/lunar_rover_ws
colcon build 
source install/setup.bash

# Python script with buttons to all needed ros nodes and rviz need to add RTAB-Map
python3 rover_launcher.py

# Test motors individualy 
// need to update to use new hardware and an Arduino
cd DiagnosticAndTesting
python3 single_motor_test.py

# Test drive chain and actuators 
// need to update to use new hardware and an Arduino
cd DiagnosticAndTesting
python3 test_drive.py

# Test Camera
cd DiagnosticAndTesting
bash test_camera_transforms.sh

# Teleop 
python3 teleop_keyboard.py

# or 

python3 controller_teleop.py
