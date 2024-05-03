#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml

def generate_launch_description():

    config_dir = os.path.join(get_package_share_directory('nmpc_astro'), 'config')
    param_config_ekf = os.path.join(config_dir, "ekf.yaml")

    with open(param_config_ekf, 'r') as f:
        param_config_ekf = yaml.safe_load(f)["ekf_filter_node"]["ros__parameters"]


    return LaunchDescription([
        Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            param_config_ekf
        ]
        )
    ])