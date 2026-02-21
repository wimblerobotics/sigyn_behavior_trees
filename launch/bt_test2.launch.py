# Copyright 2026 Wimble Robotics
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    behavior_directory = get_package_share_directory('sigyn_behavior_trees')
    xml_path = os.path.join(behavior_directory, 'config', 'bt_test2.xml')
    ld = LaunchDescription()

    behavior_node = Node(
        package='sigyn_behavior_trees',
        executable='bt_test2',
        name='bt_test2',
        parameters=[{'xml_path': xml_path}],
        respawn=False,
        output='screen')
    ld.add_action(behavior_node)

    return ld
