from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'part_assembly_for_urdf_coordinator'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='monirul',
    maintainer_email='cybernetics.rnd@gmail.com',
    description='Coordinator node for 7-DOF hybrid robotic arm',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'coordinator_node = part_assembly_for_urdf_coordinator.coordinator_node:main',
            'sbus_joint_state_publisher = part_assembly_for_urdf_coordinator.sbus_joint_state_publisher:main',
        ],
    },
)
