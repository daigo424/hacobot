#!/usr/bin/env python3
"""TurtleBot3(Waffle)をGazebo上に起動する。

turtlebot3_gazebo パッケージの turtlebot3_world.launch.py をそのまま includeするだけの
薄いラッパー。世界(ワールド)やロボットモデル自体を再実装せず、実績のある上流の
launchファイルを再利用する。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    gazebo_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    ld = LaunchDescription()
    ld.add_action(gazebo_sim_cmd)
    return ld
