#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml

def generate_launch_description():

    config_dir = os.path.join(get_package_share_directory('nmpc_astro'), 'config')
    param_config = os.path.join(config_dir, "nmpc_settings.yaml")

    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)["nmpc_settings"]["ros__parameters"]

    return LaunchDescription([

        Node(
            package='nmpc_astro',
            executable='nmpc_astro_node',
            name='nmpc_astro_node',
            output='screen',
            parameters = [
                params
            ]
        )
    ])