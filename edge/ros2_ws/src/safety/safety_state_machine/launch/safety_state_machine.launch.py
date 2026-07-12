#!/usr/bin/env python3
"""safety_state_machineノードを起動する(単体起動用)。

lifecycle nodeとして起動するのみで、configure/activateへの遷移は行わない
(他の安全層ノードとまとめて一括管理する場合は safety_bringup.launch.py を使う)。
"""
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    safety_state_machine_cmd = LifecycleNode(
        package='safety_state_machine',
        executable='safety_state_machine_node',
        name='safety_state_machine',
        namespace='',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(safety_state_machine_cmd)
    return ld
