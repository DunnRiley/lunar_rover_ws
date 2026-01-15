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

Currently having issues with the D435 camera in the ros launch.
When runnig test_camera_transforms.sh I can get point cloud but I cannot get the point cloud with the other launch from rover_launcher.py, I can get the image but thats it. 

Currently the teleop has the correct driving abilities but is missing the actuators. It includes a motor for spinning the camera but this motor dose not physicly exist yet.