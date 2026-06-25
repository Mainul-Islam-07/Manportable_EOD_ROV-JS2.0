"""Launch the coordinator node with parameters from YAML config."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('part_assembly_for_urdf_coordinator')
    params_file = os.path.join(pkg_dir, 'config', 'coordinator_params.yaml')

    coordinator_node = Node(
        package='part_assembly_for_urdf_coordinator',
        executable='coordinator_node',
        name='coordinator',
        parameters=[params_file],
        output='screen',
    )

    return LaunchDescription([coordinator_node])
