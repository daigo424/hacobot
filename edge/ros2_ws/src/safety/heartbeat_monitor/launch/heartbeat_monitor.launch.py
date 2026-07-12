#!/usr/bin/env python3
"""heartbeat_monitorノードを起動する(単体起動用)。

lifecycle nodeとして起動するのみで、configure/activateへの遷移は行わない
(`ros2 lifecycle set`で明示的に遷移させるか、単体デバッグ用に使う。
 他の安全層ノードとまとめて一括管理する場合は safety_bringup.launch.py を使う)。
"""
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    heartbeat_monitor_cmd = LifecycleNode(
        package='heartbeat_monitor',
        executable='heartbeat_monitor_node',
        name='heartbeat_monitor',
        namespace='',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(heartbeat_monitor_cmd)
    return ld
