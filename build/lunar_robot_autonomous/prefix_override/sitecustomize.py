import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/moonpie/lunar_rover_ws/install/lunar_robot_autonomous'
