import os
import glob
from setuptools import find_packages, setup

package_name = 'ros2_canbus'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: [
            'controller_config.json',
            'JS2_Motor_CANOpen_Lib_V_1_0/Motor_Mapping/*.eds',
            'JS2_Motor_CANOpen_Lib_V_1_0/Motor_Settings/*.xlsx',
            'JS2_Motor_CANOpen_Lib_V_1_0/Motor_Settings/*.json',
            'JS2_Motor_CANOpen_Lib_V_1_0/Network/*.json',
            'JS2_Motor_CANOpen_Lib_V_1_0/**/*.json',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jontro_soinik_2_0',
    maintainer_email='fiaz.tonmoy@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'robot = ros2_canbus.robot_bringup:main',
            'light = ros2_canbus.light_control_node:main',
            'fire = ros2_canbus.fire_server:main',
            'diagnostics = ros2_canbus.telemetry_udp_bridge:main',
            'beat = ros2_canbus.heartbeat_monitor_node:main',
            'battery = ros2_canbus.battery_monitor:main',
            'mode = ros2_canbus.sbus_mode_udp_bridge:main'
        ],
    },
)