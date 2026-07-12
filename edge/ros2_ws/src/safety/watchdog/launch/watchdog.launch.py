#!/usr/bin/env python3
"""watchdogノードを起動する(単体起動用)。

lifecycle nodeとして起動するのみで、configure/activateへの遷移は行わない
(他の安全層ノードとまとめて一括管理する場合は safety_bringup.launch.py を使う)。
"""
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    watchdog_cmd = LifecycleNode(
        package='watchdog',
        executable='watchdog_node',
        name='watchdog',
        namespace='',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(watchdog_cmd)
    return ld
