#!/usr/bin/env python3
"""safety_state_machineの状態を、rviz_2d_overlay_pluginsのOverlayTextとして
表示する可視化専用ノード(制御ロジックには関与しない)。

RVizのDisplaysパネルにOverlayText表示を追加し、safety/state_overlayトピックを
選択することで、画面左上にテキストとして表示できる。
"""
import collections

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from safety_msgs.msg import AnomalyEvent
from std_msgs.msg import String
from rviz_2d_overlay_msgs.msg import OverlayText

MODE_HISTORY_LEN = 8

# 各行の高さ・積み上げ位置はtext_size(make_overlay内)に対する見た目上の比率で
# 決めている。text_sizeを変えたらここも合わせて調整すること。
_LINE_HEIGHT = 36
_LINE_GAP = 5
_ANOMALY_HEIGHT = 79  # last anomalyは2行表示のため単一行の約2倍
_HISTORY_HEIGHT = 79  # historyは2行分の折り返しを見込む


def make_overlay(text, vertical_distance, height=40):
    msg = OverlayText()
    msg.text = text
    msg.width = 800
    msg.height = height
    msg.horizontal_distance = 0
    msg.vertical_distance = vertical_distance
    msg.horizontal_alignment = OverlayText.LEFT
    msg.vertical_alignment = OverlayText.TOP
    msg.line_width = 2
    msg.text_size = 9.0
    msg.font = 'Open Sans'
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
        # stall_recoveryのESCALATED等、safety/stateだけでは分からない「なぜSAFE_STOPか」を
        # 補足するために表示する(SAFE_STOP中でもsensors_healthy_かつ非E-Stopならcmd_velは
        # 中継され続けるため、state表示だけだと異常に見えてしまう)。
        self._stall_recovery_mode_pub = self.create_publisher(
            OverlayText, 'stall_recovery_mode_overlay', transient_local)
        # 現在のmodeだけでは「後退→旋回→旋回→旋回」のように何度も繰り返している
        # 経緯が分からないため、直近の遷移列を別行で表示する。
        self._mode_history_pub = self.create_publisher(
            OverlayText, 'stall_recovery_history_overlay', transient_local)
        self._mode_history = collections.deque(maxlen=MODE_HISTORY_LEN)
        # 直近のanomaly_eventの理由を表示する(SAFE_STOPになった経緯を後から見ても
        # 分かるように。stateやmodeが変わった後もこの行は最後の理由を表示し続ける)。
        self._anomaly_event_pub = self.create_publisher(
            OverlayText, 'anomaly_event_overlay', transient_local)

        self.create_subscription(
            String, 'safety/state', self._on_safety_state, transient_local)
        self.create_subscription(
            String, 'safety/stall_recovery_mode', self._on_stall_recovery_mode, transient_local)
        # anomaly_eventの発生元は基本volatile QoSで送ってくるため、transient_localを
        # 要求すると互換性が無く受信できない。ここは既定QoSで購読する。
        self.create_subscription(
            AnomalyEvent, 'safety/anomaly_event', self._on_anomaly_event, 10)

    def _on_safety_state(self, msg):
        self._safety_state_pub.publish(make_overlay(
            f'safety/state: {msg.data}',
            vertical_distance=0, height=_LINE_HEIGHT))

    def _on_stall_recovery_mode(self, msg):
        self._stall_recovery_mode_pub.publish(make_overlay(
            f'stall_recovery: {msg.data}',
            vertical_distance=_LINE_HEIGHT + _LINE_GAP, height=_LINE_HEIGHT))

        # "[stall N/M]"の付加情報は履歴では冗長なので、モード名だけ抜き出す。
        mode_name = msg.data.split(' ', 1)[0]
        self._mode_history.append(mode_name)
        self._mode_history_pub.publish(make_overlay(
            'history: ' + ' -> '.join(self._mode_history),
            vertical_distance=2 * (_LINE_HEIGHT + _LINE_GAP),
            height=_HISTORY_HEIGHT))

    def _on_anomaly_event(self, msg):
        # headerとreasonの間で改行しておく(OverlayTextは固定heightのテクスチャに
        # 描画するため、自動折り返しだけに頼ると2行目以降がheight不足で見切れる)。
        self._anomaly_event_pub.publish(make_overlay(
            f'last anomaly: [{msg.source}] {msg.severity}\n{msg.reason}',
            vertical_distance=2 * (_LINE_HEIGHT + _LINE_GAP) + _HISTORY_HEIGHT + _LINE_GAP,
            height=_ANOMALY_HEIGHT))


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
