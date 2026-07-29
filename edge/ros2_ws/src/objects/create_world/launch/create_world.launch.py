#!/usr/bin/env python3
"""ワールド生成とシミュレーター(Gazebo Sim/Harmonic)を起動する。
"""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# spawn_robot.launch.py/build_map.launch.pyのDEFAULT_MAP_DIR/DEFAULT_MAP_NAMEと
# 値を合わせる必要がある(定数重複、共有モジュール化するほどの規模ではないため)。
DEFAULT_MAP_YAML = '/data/maps/turtlebot3_world.yaml'


def _gz_sim_server_running():
    # world_supervisor.pyのクラッシュ検知と同じpgrepパターンで判定する。
    return subprocess.run(
        ['pgrep', '-f', '^gz sim server$'], stdout=subprocess.DEVNULL
    ).returncode == 0


def _reap_stale_world_processes():
    # ros2 launchは1つのアクション(gz sim)が終了しても、明示的なイベントハンドラが
    # ない限り他のアクションを道連れに終了させないため、gz simクラッシュ時は子プロセスが
    # gz sim server不在のまま残留しうる(clock_bridgeが/clockの重複publisherとして
    # 残るとハートビート遅延の一因になる)。新規起動前にまとめて掃除する。
    self_pid = os.getpid()
    result = subprocess.run(
        ['pgrep', '-f', 'ros2 launch create_world create_world.launch.py'],
        stdout=subprocess.PIPE, text=True,
    )
    for pid_str in result.stdout.split():
        pid = int(pid_str)
        if pid != self_pid:
            subprocess.run(['kill', '-9', str(pid)], stderr=subprocess.DEVNULL)

    for pattern in ('gz sim', 'parameter_bridge /clock@rosgraph_msgs', 'entity_reaper.py'):
        subprocess.run(
            ['pkill', '-9', '-f', pattern],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )


def launch_setup(context, *args, **kwargs):
    if _gz_sim_server_running():
        # 既存のgz sim serverと気付かず二重起動すると、/clock等が2系統流れてTF/時刻が壊れ、
        # explore_liteの探索打ち切りやハートビート遅延を招く。二重起動を避ける。
        return [
            LogInfo(
                msg='既存のgz sim serverを検出しました。二重起動を避けるため、'
                    '新規のGazebo/entity_reaper/clock_bridgeは起動しません。'
            ),
        ]

    _reap_stale_world_processes()

    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    world_path = os.path.join(turtlebot3_gazebo_dir, 'worlds', 'turtlebot3_world.world')

    headless = context.perform_substitution(LaunchConfiguration('headless')).lower() == 'true'
    # "-s"はGUI(gz sim gui)を起動せずサーバーのみで動かすフラグ。GUIのレンダリング負荷が
    # ホストCPUを恒常的に食い、estop_bridge等の他プロセスのDDS/Kafkaスレッドの
    # スケジューリングを圧迫するため、その対策として使う。
    gz_args = f'-s -r {world_path}' if headless else f'-r {world_path}'
    # gz_sim.launch.pyは1つのExecuteProcessから"gz sim server"+"gz sim gui"を
    # 子プロセスとして起動する(Gazebo Classicのgzserver/gzclient分離とは異なる)。
    # "-r"を付けないとHarmonicは一時停止状態で起動し、DiffDrive等のプラグインが
    # 一切publishしない(odom/tfが来ずNav2がtransform待ちで固まる)。
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items()
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

    return [gz_sim, entity_reaper, clock_bridge, map_status_log]


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'headless', default_value='false',
        description=(
            'trueにするとgz sim gui(3Dビューア)を起動せずサーバーのみで動かす。'
            'GUIのレンダリング負荷が他プロセスのスケジューリングを圧迫する事象への対策。'
        ),
    ))
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
