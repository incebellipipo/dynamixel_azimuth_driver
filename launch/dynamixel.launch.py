#!/usr/bin/env python3

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config_dynamixel_control = os.path.join(
        get_package_share_directory('mclab_dynamixel'),
        'param',
        'config.yaml'
    )

    node_dynamixel_control = launch_ros.actions.Node(
        package='mclab_dynamixel',
        executable='dynamixel_ctrl_node',
        name='dynamixel_ctrl_node',
        parameters=[config_dynamixel_control],
        output='screen'
    )

    ld = launch.LaunchDescription()

    ld.add_action(node_dynamixel_control)

    return ld