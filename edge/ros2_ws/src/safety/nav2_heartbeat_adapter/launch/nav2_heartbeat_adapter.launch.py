#!/usr/bin/env python3
"""nav2_heartbeat_adapterノードを起動する(configure/activateへの遷移は行わない)。"""
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    cmd = LifecycleNode(
        package='nav2_heartbeat_adapter',
        executable='nav2_heartbeat_adapter_node',
        name='nav2_heartbeat_adapter',
        namespace='',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(cmd)
    return ld
