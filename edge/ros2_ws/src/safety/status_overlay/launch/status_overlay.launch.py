#!/usr/bin/env python3
"""status_overlayノードを起動する(単体起動用)。"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    status_overlay_cmd = Node(
        package='status_overlay',
        executable='status_overlay_node.py',
        name='status_overlay',
        namespace='',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(status_overlay_cmd)
    return ld
