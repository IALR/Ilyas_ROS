import os
from glob import glob
from setuptools import setup

package_name = 'fleet_manager'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        # Register package with ROS2
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Fleet management system with parameterized nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Format: 'command_name = package.module:function'
            'robot_simulator = fleet_manager.robot_simulator:main',
            'fleet_monitor = fleet_manager.fleet_monitor:main',
            'emergency_stop = fleet_manager.emergency_stop:main',
        ],
    },
)