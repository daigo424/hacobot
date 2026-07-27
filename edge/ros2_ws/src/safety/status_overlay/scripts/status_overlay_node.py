#!/usr/bin/env python3
"""safety_state_machineの状態を、rviz_2d_overlay_pluginsのOverlayTextとして
表示する可視化専用ノード(制御ロジックには関与しない)。

RVizのDisplaysパネルにOverlayText表示を追加し、safety/state_overlayトピックを
選択することで、画面左上にテキストとして表示できる。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String
from rviz_2d_overlay_msgs.msg import OverlayText


def make_overlay(text):
    msg = OverlayText()
    msg.text = text
    msg.width = 400
    msg.height = 40
    msg.horizontal_distance = 0
    msg.vertical_distance = 0
    msg.horizontal_alignment = OverlayText.LEFT
    msg.vertical_alignment = OverlayText.TOP
    msg.line_width = 2
    msg.text_size = 10.0
    msg.font = 'DejaVu Sans Mono'
    return msg


class StatusOverlayNode(Node):

    def __init__(self):
        super().__init__('status_overlay')

        transient_local = QoSProfile(depth=1)
        transient_local.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # 入力がtransient_localで「変化した時だけ」publishされるのに対し、
        # 出力側が volatile のままだと、その一度きりのpublish時点でまだ購読していない側
        # (RViz起動前・診断ツール等)には二度と届かない。出力側も遅れて購読しても
        # 最新値が届くようtransient_localにする。
        self._safety_state_pub = self.create_publisher(
            OverlayText, 'safety/state_overlay', transient_local)

        self.create_subscription(
            String, 'safety/state', self._on_safety_state, transient_local)

    def _on_safety_state(self, msg):
        self._safety_state_pub.publish(
            make_overlay(f'safety/state: {msg.data}'))


def main():
    rclpy.init()
    node = StatusOverlayNode()
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
