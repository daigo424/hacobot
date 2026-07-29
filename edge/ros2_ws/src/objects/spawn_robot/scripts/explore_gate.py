#!/usr/bin/env python3
"""initial_map_seeder.pyの完了(360度旋回)を待ってから、explore_liteを起動する。

explore_liteは「costmapが少しでも使えるようになった時点」で動き出せてしまい、
それはinitial_map_seederの旋回がまだ終わっていない段階(costmapができ始めた瞬間)
でも起こりうる。そのままだとinitial_map_seederの直接cmd_vel_nav2と、explore_lite
発の指令(Nav2経由)が競合するため、initial_map_seederの完了シグナル
(initial_map_seed_doneトピック)を待ってからexplore_liteを起動する。
"""
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Empty


class ExploreGate(Node):

    def __init__(self):
        super().__init__('explore_gate')
        self._started = False
        # initial_map_seeder.py側と同じtransient_localにし、既に完了シグナルが
        # 出た後に起動しても(自身の起動が遅れても)確実に受け取れるようにする。
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Empty, 'initial_map_seed_done', self._on_seed_done, qos)

    def _on_seed_done(self, _msg):
        if self._started:
            return
        self._started = True
        ns = self.get_namespace().lstrip('/')
        self.get_logger().info('初期地図の自動生成完了を検知しました。explore_liteを起動します')
        subprocess.Popen([
            'ros2', 'launch', 'explore_lite', 'explore.launch.py',
            f'namespace:={ns}', 'use_sim_time:=true',
        ])


def main():
    rclpy.init()
    node = ExploreGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
