#!/usr/bin/env python3
"""explore_liteでフロンティア探査させながらSLAMで地図を作り、探査完了時に自動保存する。

spawn_robot.launch.pyをslam:=trueでincludeして地図生成専用ロボットを1体立て、
そこにexplore_liteのフロンティア探査ノードを追加で載せる。explore_liteは
「これ以上未探索のフロンティアが無い」と判断すると/explore/statusへ
EXPLORATION_COMPLETEをpublishするので、それを検知したmap_saver_trigger.pyが
nav2_map_serverのmap_saver_cliを呼んで地図を保存する(explore_lite自体には
保存機能が無いため)。

保存先はspawn_robot.launch.pyのDEFAULT_MAP_DIR/DEFAULT_MAP_NAMEと既定で
一致させてあるため、地図生成後は他のspawn_robot.launch.py呼び出し
(slam:=auto、既定)がそのまま自動でAMCLモード+この地図を使うようになる。
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# spawn_robot.launch.pyのDEFAULT_MAP_DIRと合わせてある(2箇所での定数重複だが、
# 別パッケージのため共有モジュール化するほどの規模ではないと判断)
DEFAULT_MAP_DIR = '/data/maps'


def launch_setup(context, *args, **kwargs):
    robot_id_str = context.perform_substitution(LaunchConfiguration('robot_id'))
    ns = f'tb3_{robot_id_str}'
    map_name = context.perform_substitution(LaunchConfiguration('map_name'))
    map_path = os.path.join(DEFAULT_MAP_DIR, map_name)

    spawn_robot_dir = get_package_share_directory('spawn_robot')
    spawn_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(spawn_robot_dir, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'robot_id': robot_id_str,
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'z_pose': LaunchConfiguration('z_pose'),
            'slam': 'true',
            'auto_seed_map': 'true',
            'watchdog_timeout_ms': LaunchConfiguration('watchdog_timeout_ms'),
        }.items(),
    )

    explore_dir = get_package_share_directory('explore_lite')
    explore_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(explore_dir, 'launch', 'explore.launch.py')
        ),
        launch_arguments={
            'namespace': ns,
            'use_sim_time': 'true',
        }.items(),
    )

    map_saver_trigger_cmd = Node(
        package='spawn_robot',
        executable='map_saver_trigger.py',
        namespace=ns,
        arguments=[map_path],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # hacobot_view.rvizは全トピックが"tb3_01"決め打ちのため、対象namespaceへ一括置換した
    # コピーを使う。/tfはtf2の仕様上絶対パスになるため別途remapも必要(spawn_robot.launch.py参照)。
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', build_rviz_config(ns)],
        remappings=[
            ('/tf', f'/{ns}/tf'),
            ('/tf_static', f'/{ns}/tf_static'),
        ],
        namespace=ns,
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [spawn_cmd, explore_cmd, map_saver_trigger_cmd, rviz_cmd]


def build_rviz_config(ns):
    rviz_src_path = os.path.join(
        get_package_share_directory('nav2_bringup_custom'), 'rviz', 'hacobot_view.rviz'
    )
    with open(rviz_src_path, 'r') as f:
        content = f.read()
    content = content.replace('tb3_01', ns)

    rviz_tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.rviz', delete=False)
    rviz_tmp.write(content)
    rviz_tmp.close()

    return rviz_tmp.name


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'robot_id', default_value='builder',
        description='地図生成専用ロボットのID(tb3_<robot_id>という名前空間になる)'))
    # turtlebot3_worldでは(0.0, 0.0)は壁際で物理エンジンに弾き出されるため、
    # spawn_robot.launch.pyと同じ壁の無い座標をデフォルトにする
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='-2.0'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='-0.5'))
    ld.add_action(DeclareLaunchArgument('z_pose', default_value='0.01'))
    ld.add_action(DeclareLaunchArgument(
        'map_name', default_value='turtlebot3_world',
        description=(
            '保存する地図ファイル名(拡張子無し)。spawn_robot.launch.pyの既定の'
            '読み込み先と合わせる場合はturtlebot3_worldのままにする'
        ),
    ))
    ld.add_action(DeclareLaunchArgument(
        'rviz', default_value='true',
        description='探査の様子(地図・ロボット)をRVizで表示するか',
    ))
    ld.add_action(DeclareLaunchArgument(
        'watchdog_timeout_ms', default_value='1000',
        description=(
            'このロボットのWatchdogセンサー途絶しきい値[ms]。既定の実機値(500)だと'
            'Gazeboのレンダリング負荷で誤ってSAFE_STOPに入り探査が止まるため緩めている。'
        ),
    ))

    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
