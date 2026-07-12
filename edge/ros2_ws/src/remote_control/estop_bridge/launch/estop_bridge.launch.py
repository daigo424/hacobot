#!/usr/bin/env python3
"""estop_bridgeノードを起動する(configure/activateへの遷移は行わない)。"""
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    cmd = LifecycleNode(
        package='estop_bridge',
        executable='estop_bridge_node',
        name='estop_bridge',
        namespace='',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(cmd)
    return ld
