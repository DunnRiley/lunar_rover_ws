from setuptools import setup
import os
from glob import glob

package_name = 'lunar_robot_autonomous'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py') if os.path.exists('launch') else []),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') if os.path.exists('config') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Autonomous navigation for lunar rover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'unified_navigator = lunar_robot_autonomous.unified_navigator:main',
            'multi_waypoint_navigator = lunar_robot_autonomous.multi_waypoint_navigator:main',
        ],
    },
)
