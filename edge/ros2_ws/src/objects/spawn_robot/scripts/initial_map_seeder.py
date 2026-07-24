#!/usr/bin/env python3
"""スポーン直後の一定時間だけ、簡単な円運動を自動でcmd_vel_nav2へ送り、SLAMの初期地図を育てる。

nav2_costmap_2dは地図が完全に空(width/height=0)だと経路計画ができず、経路が無いと
ロボットは動けず、動けないとslam_toolboxの地図も育たない、という鶏と卵のデッドロックが
あるため、起動直後にこのノードが自律的にロボットを動かして最初の地図を作る。
cmd_vel_nav2はsafety_state_machineを経由するため、センサー異常時は通常通り停止する。
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from visualization_msgs.msg import Marker

SEED_DURATION_SEC = 30.0
PUBLISH_PERIOD_SEC = 0.1
STATUS_MARKER_ID = 0


class InitialMapSeeder(Node):

    def __init__(self):
        super().__init__('initial_map_seeder')
        self._pub = self.create_publisher(TwistStamped, 'cmd_vel_nav2', 10)
        # RVizで「マッピング中」かどうかを見えるようにする、ロボット追従のテキストマーカー
        self._marker_pub = self.create_publisher(Marker, 'mapping_status', 10)
        # コンストラクタ時点では/clockをまだ一度も受信しておらずnow()が0を返すことがあり、
        # それを基準にすると次のタイマー周期で経過時間が一気に跳ね上がってしまうため、
        # 基準時刻は最初のタイマー発火時に確定させる
        self._start_time = None
        self._timer = self.create_timer(PUBLISH_PERIOD_SEC, self._on_timer)

    def _publish_status_marker(self, active):
        marker = Marker()
        marker.header.frame_id = 'base_footprint'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'mapping_status'
        marker.id = STATUS_MARKER_ID
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD if active else Marker.DELETE
        marker.pose.position.z = 0.4
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.15
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.text = 'マッピング中...'
        self._marker_pub.publish(marker)

    def _on_timer(self):
        now = self.get_clock().now()
        if self._start_time is None:
            if now.nanoseconds == 0:
                # /clockをまだ一度も受信していない(use_sim_timeでは起こりうる)。
                # 有効な時刻が届くまで基準時刻の確定を待つ
                return
            self._start_time = now
            return

        elapsed_sec = (now - self._start_time).nanoseconds / 1e9
        if elapsed_sec >= SEED_DURATION_SEC:
            self._pub.publish(TwistStamped())
            self._publish_status_marker(active=False)
            self.get_logger().info('初期地図の自動生成を終了しました')
            self._timer.cancel()
            return

        cmd = TwistStamped()
        cmd.header.stamp = now.to_msg()
        cmd.twist.linear.x = 0.15
        cmd.twist.angular.z = 0.4
        self._pub.publish(cmd)
        self._publish_status_marker(active=True)


def main():
    rclpy.init()
    node = InitialMapSeeder()
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
