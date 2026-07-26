#!/usr/bin/env python3
"""ワールド生成とシミュレーター(Gazebo Sim/Harmonic)を起動する。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# spawn_robot.launch.py/build_map.launch.pyのDEFAULT_MAP_DIR/DEFAULT_MAP_NAMEと
# 値を合わせる必要がある(定数重複、共有モジュール化するほどの規模ではないため)。
DEFAULT_MAP_YAML = '/data/maps/turtlebot3_world.yaml'


def generate_launch_description():
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    world_path = os.path.join(turtlebot3_gazebo_dir, 'worlds', 'turtlebot3_world.world')

    # gz_sim.launch.pyは1つのExecuteProcessから"gz sim server"+"gz sim gui"を
    # 子プロセスとして起動する(Gazebo Classicのgzserver/gzclient分離とは異なる)。
    # "-r"を付けないとHarmonicは一時停止状態で起動し、DiffDrive等のプラグインが
    # 一切publishしない(odom/tfが来ずNav2がtransform待ちで固まる)。
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # spawn_robotが生成したロボットが、対応するノードが無いままGazebo上に取り残される
    # (SIGKILL等でspawn_robot自身の後片付けが実行されない)ことがあるため、ワールドと
    # 同じ寿命の監視ノードで定期的に回収する。spawn_robotではなくこちらに紐づけるのは、
    # ワールド(gz sim server)が生きている限り監視を続けたいため。
    entity_reaper = Node(
        package='spawn_robot',
        executable='entity_reaper.py',
        output='screen',
    )

    # /clockはロボット数に依らずワールドに1本だけの共有クロックであるべきなので、
    # (spawn_robot側の per-robot bridge には含めていない)、ここでワールドと同じ
    # 寿命で一度だけ橋渡しする。
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    if os.path.exists(DEFAULT_MAP_YAML):
        map_status_log = LogInfo(
            msg=f'既存の地図を検出しました({DEFAULT_MAP_YAML})。'
                'spawn_robot.launch.pyはslam引数が既定(auto)のままなら'
                'この地図を使うAMCLモードで起動します。'
        )
    else:
        map_status_log = LogInfo(
            msg=f'地図が見つかりません({DEFAULT_MAP_YAML})。'
                'spawn_robot.launch.pyはslam引数が既定(auto)のままなら'
                'SLAMモードで起動します。地図を作りたい場合は'
                'build_map.launch.pyを使ってください。'
        )

    ld = LaunchDescription()
    ld.add_action(gz_sim)
    ld.add_action(entity_reaper)
    ld.add_action(clock_bridge)
    ld.add_action(map_status_log)
    return ld
