from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'lunar_robot_hardware'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rileydunn',
    maintainer_email='you@example.com',
    description='Lunar robot Arduino hardware interface',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller_node = lunar_robot_hardware.motor_controller:main',
            'simple_odom_publisher = lunar_robot_hardware.simple_odom:main',
            'arduino_motor_controller = lunar_robot_hardware.arduino_motor_controller:main',
            'arduino_teleop = lunar_robot_hardware.arduino_teleop:main',
            'controller_teleop = lunar_robot_hardware.arduino_teleop_controller:main',
        ],
    },
)
