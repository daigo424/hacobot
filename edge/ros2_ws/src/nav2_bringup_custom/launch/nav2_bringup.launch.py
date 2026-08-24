#!/usr/bin/env python3
"""SLAM Toolbox + Nav2(Planner/Controller)スタックと、hacobotフェイルセーフ層を起動する。

nav2_bringup パッケージの bringup_launch.py を slam:=true で includeする薄いラッパー。
slam:=true にすることで、事前地図+AMCLではなく、走行しながら地図を作るオンラインSLAM
(slam_toolbox)を自己位置推定源として使う構成になる(nav2_bringupの標準機能)。
params_file には、本家Nav2既定値(params/defaults/nav2_params.yaml)をベースにhacobot固有の
差分だけをコード上で上書きしたYAML(nav2_params_builder.build_nav2_params_yaml()参照)を渡す。

フェイルセーフ層との統合のため、以下2点を追加している:
1. Nav2の最終的な速度指令の出力先を /cmd_vel_nav2 にリマップする(SetRemap)。
   実際にロボットへ送る /cmd_vel は safety_state_machine が中継/override する。
   Nav2自身に /cmd_vel を直接掴ませない。
2. safety_bringup(heartbeat_monitor, safety_state_machine, watchdog,
   nav2_heartbeat_adapter, estop_bridge)を同時に起動する。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap
from nav2_params_builder import build_nav2_params_yaml
from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    safety_bringup_dir = get_package_share_directory('safety_bringup')
    default_params_file = build_nav2_params_yaml()

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file', default=default_params_file)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Gazeboのシミュレーション時刻を使うか')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', default_value=default_params_file,
        description='Nav2/SLAM Toolboxに渡すパラメータYAML')

    gazebo_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x_pose': '-2.0',
            'y_pose': '-0.5',
        }.items()
    )

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            # 'slam'はbringup_launch.py内部でPythonExpression(eval)による条件分岐に使われるため
            # 'true'ではなくPythonの真偽値リテラルと同じ 'True' (先頭大文字)でなければならない。
            # ('autostart'はノードパラメータとして渡るだけなので小文字'true'のままでよい)
            'slam': 'True',
            # slam:=Trueの場合map引数はbringup_launch.py内で未使用だが、
            # デフォルト値を持たない必須引数のため空文字を明示的に渡す
            'map': '',
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
        }.items()
    )

    # bringup_launch.pyの中身(velocity_smoother等)は編集できないので、
    # このスコープ内で起動される全ノードの"cmd_vel"という名前のトピックを
    # 一括で"cmd_vel_nav2"にリマップすることで、Nav2に直接/cmd_velを出させない
    nav2_bringup_remapped = GroupAction(
        actions=[
            SetRemap(src='cmd_vel', dst='cmd_vel_nav2'),
            nav2_bringup_cmd,
        ]
    )

    safety_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(safety_bringup_dir, 'launch', 'safety_bringup.launch.py')
        )
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('nav2_bringup_custom'),
        'rviz', 'view.rviz'
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(gazebo_sim_cmd)
    ld.add_action(nav2_bringup_remapped)
    ld.add_action(safety_bringup_cmd)
    ld.add_action(rviz_cmd)
    return ld
