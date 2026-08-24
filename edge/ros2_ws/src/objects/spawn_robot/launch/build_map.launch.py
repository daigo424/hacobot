#!/usr/bin/env python3
"""SLAMで地図を作りながら、explore_liteによる自動探査、またはキーボードでの手動操縦で走らせる。

spawn_robot.launch.pyをslam:=trueでincludeして地図生成専用ロボットを1体立てる。
exploration_mode:=auto(既定)ではexplore_liteのフロンティア探査ノードを追加で載せ、
「これ以上未探索のフロンティアが無い」と判断すると/explore/statusへ
EXPLORATION_COMPLETEをpublishするので、それを検知したmap_saver_trigger.pyが
nav2_map_serverのmap_saver_cliを呼んで地図を保存する(explore_lite自体には
保存機能が無いため)。

exploration_mode:=manualではexplore_liteを起動せず、代わりに別ターミナルで
teleop_twist_keyboardを起動してもらう(キーボード入力はttyの生読み取りが要るため、
ros2 launch配下のノードとして埋め込むと標準入力が正しく渡らないことがある、という
既知の制約を踏まえた構成)。保存はEXPLORATION_COMPLETEが来ないため自動発火しない。
save_map_triggerトピックへ空メッセージをpublishすると同じ保存処理が手動で起動できる。

保存先はspawn_robot.launch.pyのDEFAULT_MAP_DIR/DEFAULT_MAP_NAMEと既定で
一致させてあるため、地図生成後は他のspawn_robot.launch.py呼び出し
(slam:=auto、既定)がそのまま自動でAMCLモード+この地図を使うようになる。
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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
            'auto_seed_map': LaunchConfiguration('auto_seed_map'),
            'watchdog_timeout_ms': LaunchConfiguration('watchdog_timeout_ms'),
            'heartbeat_lease_ms': LaunchConfiguration('heartbeat_lease_ms'),
        }.items(),
    )

    is_auto = PythonExpression(["'", LaunchConfiguration('exploration_mode'), "' == 'auto'"])
    is_manual = PythonExpression(["'", LaunchConfiguration('exploration_mode'), "' == 'manual'"])

    # explore_liteはcostmapが少しでも使えるようになった時点で動き出せてしまい、
    # それはinitial_map_seederの360度旋回が終わる前でも起こりうる。ここで直接
    # includeせず、旋回完了(initial_map_seed_doneトピック)を待つexplore_gate.py
    # 経由でexplore_liteを起動する(spawn_robot.launch.py参照)。
    explore_cmd = Node(
        package='spawn_robot',
        executable='explore_gate.py',
        namespace=ns,
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(is_auto),
    )

    # teleop_twist_keyboardはtty入力の生読み取りが要るため、ros2 launch配下の
    # ノードとしては起動しない(標準入力が正しく渡らないことがある既知の制約)。
    # 別ターミナルで実行してもらうためのコマンドをここに案内表示する。
    manual_mode_hint = LogInfo(
        msg=(
            '手動操縦モードです。別のターミナルで以下を実行してください:\n'
            'ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args '
            f'-p stamped:=true -r cmd_vel:=/{ns}/cmd_vel_nav2_raw\n'
            '走行を終えたら地図を保存: '
            f'ros2 topic pub --once /{ns}/save_map_trigger std_msgs/msg/Empty {{}}'
        ),
        condition=IfCondition(is_manual),
    )

    # x_pose/y_poseはこのロボットのSLAM開始点(=mapフレーム原点に対応するGazebo世界座標)。
    # map_saver_trigger.pyが地図保存時にサイドカーファイルへ記録し、spawn_robot.launch.py
    # のAMCLモードがそこから初期位置を自動算出する(spawn_robot.launch.py参照)。
    map_saver_trigger_cmd = Node(
        package='spawn_robot',
        executable='map_saver_trigger.py',
        namespace=ns,
        arguments=[map_path, LaunchConfiguration('x_pose'), LaunchConfiguration('y_pose')],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # spawn_robot_view.rvizは全トピックが"tb3_01"決め打ちのため、対象namespaceへ一括置換した
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

    return [spawn_cmd, explore_cmd, manual_mode_hint, map_saver_trigger_cmd, rviz_cmd]


def build_rviz_config(ns):
    rviz_src_path = os.path.join(
        get_package_share_directory('spawn_robot'), 'rviz', 'spawn_robot_view.rviz'
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
        'auto_seed_map', default_value='true',
        description=(
            '起動直後にinitial_map_seeder.pyが自動で円運動させ、地図が空で'
            'Nav2が経路計画できないデッドロックを避けるか。falseにすると'
            'この自動走行(いわゆる「最初にくるくる回る」動き)が無くなるが、'
            '手動操縦(exploration_mode:=manual)等で自分で最初の一歩を'
            '動かす必要がある'
        ),
    ))
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
        'exploration_mode', default_value='auto',
        description=(
            "'auto'(既定、explore_liteによる自動フロンティア探査)または"
            "'manual'(teleop_twist_keyboardによる手動操縦。別ターミナルでの"
            '起動が必要、詳細は起動時のログ案内を参照)'
        ),
    ))
    ld.add_action(DeclareLaunchArgument(
        'watchdog_timeout_ms', default_value='1000',
        description=(
            'このロボットのWatchdogセンサー途絶しきい値[ms]。既定の実機値(500)だと'
            'Gazeboのレンダリング負荷で誤ってSAFE_STOPに入り探査が止まるため緩めている。'
        ),
    ))
    ld.add_action(DeclareLaunchArgument(
        'heartbeat_lease_ms', default_value='2000',
        description=(
            'heartbeat_monitor/estop_bridge/nav2_heartbeat_adapter間のDDS Liveliness QoSの'
            'lease duration[ms]。既定の実機値(300)だと、build_map中はGazeboのシミュレーション'
            '計算負荷(GUIの有無に関わらず、物理演算・センサーのレイトレーシング自体)が'
            'ホストのスケジューリングを瞬間的に(実測で最大420ms程度)圧迫し、誤ってSAFE_STOPに'
            '入るため、実測値に対して約5倍の余裕を見て緩めている。'
        ),
    ))

    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
