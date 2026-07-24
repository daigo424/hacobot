#!/usr/bin/env python3
"""SLAM Toolbox + Nav2(Planner/Controller)スタックと、hacobotフェイルセーフ層を起動する。

nav2_bringup パッケージの bringup_launch.py を slam:=true で includeする薄いラッパー。
slam:=true にすることで、事前地図+AMCLではなく、走行しながら地図を作るオンラインSLAM
(slam_toolbox)を自己位置推定源として使う構成になる(nav2_bringupの標準機能)。
params_file には本パッケージの params/nav2_params.yaml (TurtleBot3 Waffle向けにチューニング
+ slam_toolbox設定を含む)を渡す。

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


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    safety_bringup_dir = get_package_share_directory('safety_bringup')
    default_params_file = os.path.join(
        get_package_share_directory('nav2_bringup_custom'),
        'params',
        'nav2_params.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file', default=default_params_file)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Gazeboのシミュレーション時刻を使うか')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', default_value=default_params_file,
        description='Nav2/SLAM Toolboxに渡すパラメータYAML')

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

    # nav2_bringupのslam_launch.pyはslam_toolbox付属のautostart機構に頼っているが、
    # 実測では自動発火しないことが多いため、configure/activateを明示的に呼ぶ
    # (spawn_robot.launch.pyと同じ回避策。詳細はactivate_slam_toolbox.py参照)
    activate_slam_toolbox_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_bringup_custom', 'activate_slam_toolbox.py', '/slam_toolbox'],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(nav2_bringup_remapped)
    ld.add_action(safety_bringup_cmd)
    ld.add_action(activate_slam_toolbox_cmd)
    return ld
