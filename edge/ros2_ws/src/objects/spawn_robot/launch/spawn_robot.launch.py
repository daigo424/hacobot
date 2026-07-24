#!/usr/bin/env python3
"""複数体のTurtleBot3を、ロボットごとに完全独立したNav2+フェイルセーフ層で起動する。

各ロボットは専用の/tb3_0N/tf・独立したNav2/SLAMを持つ(nav2_bringupのnamespace機構を使用)。
そのためRVizで複数ロボットを1つのFixed Frameに同時表示することはできない
(ロボットごとに別のmapフレームを持つため、共通の親フレームが無い)。
"""
import colorsys
import hashlib
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap

def launch_setup(context, *args, **kwargs):
    robot_id_str = context.perform_substitution(LaunchConfiguration('robot_id'))
    ns = f"tb3_{robot_id_str}"

    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')

    model_folder = 'turtlebot3_' + os.environ.get('TURTLEBOT3_MODEL', 'waffle')

    # robot_state_publisher用にturtlebot3_gazeboのURDFを使う
    # (turtlebot3_descriptionのURDFにはGazeboのセンサー/駆動プラグインが無い)
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'urdf', f'{model_folder}.urdf'
    )
    with open(urdf_path, 'r') as f:
        urdf_content = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': urdf_content,
        }]
    )

    # 実際にスポーンする実体はturtlebot3_gazebo本家のmodel.sdfを使う
    sdf_tmp = build_spawn_sdf(model_folder, robot_id_str)
    bridge_yaml_tmp = build_bridge_yaml(model_folder, robot_id_str)

    spawn_turtlebot3 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', ns,
            '-file', sdf_tmp.name,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
        ],
        output='screen'
    )

    # ros_topic_nameは相対名のままなので、PushRosNamespace(ns)配下に置くだけで
    # /tb3_0N/scan等に自動的に収まる(gz_topic_nameは build_bridge_yaml 側で
    # ロボットごとに接頭辞を付けて衝突を避けている)
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_yaml_tmp.name}'],
        output='screen',
    )

    # image_bridgeはGZ側・ROS側で同一のトピック名を使うため、ここだけは
    # robot_group(PushRosNamespace)の外に置いて接頭辞付き名をそのまま渡す
    # (中に入れると二重にnamespaceが積まれる)
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[f'{ns}/camera/image_raw'],
        output='screen',
    )

    # --- Nav2(SLAM Toolbox込み) ---
    # bringup_launch.pyは自分自身でPushRosNamespaceするため、下のrobot_group(自前の
    # PushRosNamespace)には含めない(二重にnamespaceが積まれる)。
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_params_file = os.path.join(
        get_package_share_directory('nav2_bringup_custom'), 'params', 'nav2_params.yaml'
    )
    nav2_cmd = GroupAction([
        # Nav2の相対"cmd_vel"出力を"cmd_vel_nav2"に付け替える(safety_state_machineが中継する)
        SetRemap(src='cmd_vel', dst='cmd_vel_nav2'),
        # nav2_params.yaml内の絶対パス/scan(costmap各層・slam_toolbox等、計6箇所)を相対化
        SetRemap(src='/scan', dst='scan'),
        # bringup_launch.pyはnav2_containerにだけ/tf remapを渡していて、内部でincludeする
        # slam_launch.pyには渡していない(upstream側の抜け漏れ)。ここで指定すれば両方効く。
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),
        # slam_toolboxは/tf同様、/map・/map_metadataも絶対パスで固定publishするため、
        # 複数ロボット共存時に全ロボットのSLAMが同じ/mapに衝突する。ここで相対化する。
        SetRemap(src='/map', dst='map'),
        SetRemap(src='/map_metadata', dst='map_metadata'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'namespace': ns,
                'use_namespace': 'true',
                # 内部でPythonのeval()条件分岐に使われるため'True'(先頭大文字)が必須
                'slam': 'True',
                # slam:=Trueでは未使用だが、デフォルト値の無い必須引数なので空文字を渡す
                'map': '',
                'use_sim_time': 'true',
                'params_file': nav2_params_file,
                'autostart': 'true',
            }.items(),
        ),
    ])

    # --- フェイルセーフ層(safety_bringup) ---
    safety_bringup_dir = get_package_share_directory('safety_bringup')
    safety_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(safety_bringup_dir, 'launch', 'safety_bringup.launch.py')
        )
    )

    # 地図が空(0x0)だとNav2は経路計画できず、経路が無いとロボットは動けず、動けないと
    # slam_toolboxの地図も育たない、というデッドロックがあるため、スポーン直後の一定時間だけ
    # 自動でロボットを動かして初期地図を育てる。cmd_vel_nav2経由なのでsafety_state_machineの
    # 制御(センサー異常時は止まる)は通常通り効く。
    initial_map_seeder_cmd = Node(
        package='spawn_robot',
        executable='initial_map_seeder.py',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('auto_seed_map')),
    )

    # フェイルセーフ層5ノードは/safety/*・/cmd_vel等を絶対パスで決め打ちしているため、
    # PushRosNamespaceの前にSetRemapで相対化する(/scan等はwatchdog/adapterが見るため)。
    robot_group = GroupAction([
        PushRosNamespace(ns),
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),
        SetRemap(src='/scan', dst='scan'),
        SetRemap(src='/camera/image_raw', dst='camera/image_raw'),
        SetRemap(src='/imu', dst='imu'),
        SetRemap(src='/local_costmap/costmap', dst='local_costmap/costmap'),
        SetRemap(src='/safety/anomaly_event', dst='safety/anomaly_event'),
        SetRemap(src='/safety/recovery_command', dst='safety/recovery_command'),
        SetRemap(src='/safety/heartbeat/nav2', dst='safety/heartbeat/nav2'),
        SetRemap(src='/safety/heartbeat/comm_bridge', dst='safety/heartbeat/comm_bridge'),
        SetRemap(src='/safety/state', dst='safety/state'),
        SetRemap(src='/safety/sensors_ok', dst='safety/sensors_ok'),
        SetRemap(src='/cmd_vel_nav2', dst='cmd_vel_nav2'),
        SetRemap(src='/cmd_vel', dst='cmd_vel'),
        robot_state_publisher,
        gz_bridge,
        safety_cmd,
        initial_map_seeder_cmd,
    ])

    # nav2_bringupのslam_launch.pyはslam_toolbox付属のautostart機構に頼っているが、
    # 実測では自動発火しないことが多いため、configure/activateを明示的に呼ぶ
    # (activate_slam_toolbox.py内で対象ノードが上がるまでリトライする)
    activate_slam_toolbox_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_bringup_custom', 'activate_slam_toolbox.py', f'/{ns}/slam_toolbox'],
        output='screen',
    )

    # 前回異常終了で同名エンティティが残っていると"already exists"で失敗するため、
    # スポーン前にベストエフォートで削除してからspawnする。
    # delete_entity_safely.pyが削除前後に物理エンジンを一時停止/再開する
    # (Gazebo Classicのdelete時segfault対策)。
    delete_existing_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'spawn_robot', 'delete_entity_safely.py', ns],
        output='screen',
    )
    spawn_after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=delete_existing_cmd,
            on_exit=[spawn_turtlebot3],
        )
    )

    # OnProcessExitは全体シャットダウン中に新規アクションをスキップすることがあるため、
    # シャットダウン専用のOnShutdown+同期subprocess.runで呼ぶ。
    def _delete_entity_on_shutdown(event, context):
        subprocess.run(
            ['ros2', 'run', 'spawn_robot', 'delete_entity_safely.py', ns],
            timeout=10,
        )

    clean_up_action = RegisterEventHandler(
        OnShutdown(on_shutdown=_delete_entity_on_shutdown)
    )

    return [
        robot_group,
        nav2_cmd,
        image_bridge,
        activate_slam_toolbox_cmd,
        delete_existing_cmd,
        spawn_after_cleanup,
        clean_up_action,
    ]


# gz sim(Harmonic)はモデルインスタンスごとにgzトピックを自動分離しない
# (同一SDFから2体スポーンすると両方とも/scan・/cmd_vel等に衝突することを実測で確認済み)。
# spawnするSDF内のトピック名と、ROS側にブリッジするgzトピック名の両方に、この接頭辞で
# 揃えて明示的に分離する。
GZ_TOPIC_TAGS = ('topic', 'odom_topic', 'tf_topic', 'camera_info_topic')


def build_spawn_sdf(model_folder, robot_id_str):
    ns = f"tb3_{robot_id_str}"
    sdf_src_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'models', model_folder, 'model.sdf'
    )
    tree = ET.parse(sdf_src_path)
    root = tree.getroot()

    # 車体色をrobot_idから生成(色相をずらし、別IDなら必ず色が離れる)。
    # URDFのmaterial colorはGazebo変換で反映されないため、SDFのambient/diffuseを直接書き換える。
    hue_seed = int(hashlib.sha1(robot_id_str.encode()).hexdigest(), 16) % (2 ** 32)
    hue = (hue_seed * 0.6180339887498949) % 1.0
    r, g, b = colorsys.hsv_to_rgb(hue, 0.65, 0.95)
    for visual in root.iter('visual'):
        if visual.get('name') == 'base_visual':
            material = visual.find('material')
            if material is not None:
                for tag_name in ('ambient', 'diffuse'):
                    el = material.find(tag_name)
                    if el is not None:
                        el.text = f'{r:.3f} {g:.3f} {b:.3f} 1.0'
            # visual名が共通のままだとGazeboがマテリアルを使い回し、2体目以降の色が変わらない
            visual.set('name', f'base_visual_{ns}')

    for tag_name in GZ_TOPIC_TAGS:
        for el in root.iter(tag_name):
            if el.text:
                el.text = f'{ns}/{el.text.lstrip("/")}'

    sdf_content = '<?xml version="1.0" ?>\n' + ET.tostring(root, encoding='unicode')
    sdf_tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False)
    sdf_tmp.write(sdf_content)
    sdf_tmp.close()

    return sdf_tmp


def build_bridge_yaml(model_folder, robot_id_str):
    ns = f"tb3_{robot_id_str}"
    bridge_src_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'params', f'{model_folder}_bridge.yaml'
    )
    with open(bridge_src_path, 'r') as f:
        entries = yaml.safe_load(f)

    # clockはワールド全体で共有すべき単一のクロックなので、ロボットごとの接頭辞を付けず
    # ここでは除外する(create_world.launch.py側でワールド共通のbridgeとして扱う)
    per_robot_entries = [e for e in entries if e['gz_topic_name'] != 'clock']
    for entry in per_robot_entries:
        entry['gz_topic_name'] = f'{ns}/{entry["gz_topic_name"].lstrip("/")}'

    bridge_tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.safe_dump(per_robot_entries, bridge_tmp)
    bridge_tmp.close()

    return bridge_tmp


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('robot_id', default_value='01'))
    # turtlebot3_worldでは(0.0, 0.0)は壁際で物理エンジンに弾き出されるため、
    # upstreamと同じ壁の無い座標をデフォルトにする
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='-2.0'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='-0.5'))
    ld.add_action(DeclareLaunchArgument('z_pose', default_value='0.01'))
    ld.add_action(DeclareLaunchArgument('auto_seed_map', default_value='true'))

    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
