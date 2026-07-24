#!/usr/bin/env python3
"""ワールド生成とシミュレーター(Gazebo)を起動する。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    gzserver  = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': os.path.join(turtlebot3_gazebo_dir, 'worlds', 'turtlebot3_world.world')}.items()
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        )
    )

    # spawn_robotが生成したロボットが、対応するノードが無いままGazebo上に取り残される
    # (SIGKILL等でspawn_robot自身の後片付けが実行されない)ことがあるため、ワールドと
    # 同じ寿命の監視ノードで定期的に回収する。spawn_robotではなくこちらに紐づけるのは、
    # ワールド(gzserver)が生きている限り監視を続けたいため。
    entity_reaper = Node(
        package='spawn_robot',
        executable='entity_reaper.py',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(entity_reaper)
    return ld
