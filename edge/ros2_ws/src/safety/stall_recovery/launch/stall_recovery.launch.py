#!/usr/bin/env python3
"""stall_recoveryノードを起動する(単体起動用)。

lifecycle nodeとして起動するのみで、configure/activateへの遷移は行わない
(他の安全層ノードとまとめて一括管理する場合は safety_bringup.launch.py を使う)。
"""
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    stall_recovery_cmd = LifecycleNode(
        package='stall_recovery',
        executable='stall_recovery_node',
        name='stall_recovery',
        namespace='',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(stall_recovery_cmd)
    return ld
