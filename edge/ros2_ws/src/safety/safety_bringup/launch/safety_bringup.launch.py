#!/usr/bin/env python3
"""hacobot フェイルセーフ層5ノードを一括起動する。

heartbeat_monitor, safety_state_machine, watchdog, nav2_heartbeat_adapter,
estop_bridgeをlifecycle nodeとして起動し、nav2_lifecycle_manager
(Nav2専用ではなく任意のlifecycle nodeを管理できる汎用パッケージ。nav2_bringup自身が
slam_launch.py/navigation_launch.pyで使っているのと同じ仕組みを流用)で
自動的にconfigure→activateまで遷移させる。

注意: 安全層のタイミング判定(ハートビート途絶検知など)はシミュレーション速度に
左右されるべきではない(実機での応答性要件をそのまま反映すべき)という設計判断により、
これらのノードは use_sim_time を設定せず、常に実時間(wall clock)で動作させる。
"""
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node

LIFECYCLE_NODE_NAMES = [
    'heartbeat_monitor',
    'safety_state_machine',
    'watchdog',
    'nav2_heartbeat_adapter',
    'estop_bridge',
]


def generate_launch_description():
    heartbeat_monitor_cmd = LifecycleNode(
        package='heartbeat_monitor',
        executable='heartbeat_monitor_node',
        name='heartbeat_monitor',
        namespace='',
        output='screen',
    )

    safety_state_machine_cmd = LifecycleNode(
        package='safety_state_machine',
        executable='safety_state_machine_node',
        name='safety_state_machine',
        namespace='',
        output='screen',
    )

    watchdog_cmd = LifecycleNode(
        package='watchdog',
        executable='watchdog_node',
        name='watchdog',
        namespace='',
        output='screen',
    )

    nav2_heartbeat_adapter_cmd = LifecycleNode(
        package='nav2_heartbeat_adapter',
        executable='nav2_heartbeat_adapter_node',
        name='nav2_heartbeat_adapter',
        namespace='',
        output='screen',
    )

    estop_bridge_cmd = LifecycleNode(
        package='estop_bridge',
        executable='estop_bridge_node',
        name='estop_bridge',
        namespace='',
        output='screen',
        parameters=[{
            # このdev環境固有の到達先(k3dのnodeportリスナー、infra/k8s/kafka/kafka-cluster.yaml
            # 参照)。ネイティブDocker Engine(Docker Desktopではない)を使っている場合のみ
            # 127.0.0.1経由でros2_nav2_containerからk3d内のKafkaへ実際に到達できる
            # (Docker Desktop使用時は別ネットワーク名前空間のため到達不可)。
            # 本番環境やk3d内にPodとしてデプロイする場合は、estop_bridge_node.cppのコード側デフォルト
            # (hacobot-kafka-kafka-bootstrap.kafka.svc.cluster.local:9092、
            # in-cluster DNS)をそのまま使えばよく、このオーバーライドは不要になる。
            'kafka_brokers': '127.0.0.1:30092',
            'kafka_host': '127.0.0.1',
            'kafka_port': 30092,
            # assume_healthyバイパスは不要になった(実際にTCP到達性チェックが機能する)。
            'assume_healthy': False,
        }],
    )

    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_safety',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': LIFECYCLE_NODE_NAMES,
            # 安全層のノードはnav2_util::LifecycleNodeのbond機構を実装していないため、
            # bondタイムアウトを無効化する(既定4秒のままだと全ノードが起動失敗扱いになる)
            'bond_timeout': 0.0,
        }],
    )

    ld = LaunchDescription()
    ld.add_action(heartbeat_monitor_cmd)
    ld.add_action(safety_state_machine_cmd)
    ld.add_action(watchdog_cmd)
    ld.add_action(nav2_heartbeat_adapter_cmd)
    ld.add_action(estop_bridge_cmd)
    ld.add_action(lifecycle_manager_cmd)
    return ld
