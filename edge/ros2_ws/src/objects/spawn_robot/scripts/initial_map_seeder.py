#!/usr/bin/env python3
"""スポーン直後、slam_toolboxの初回mapを待ってからその場で360度旋回し、
explore_gate.pyに引き継ぐ。

nav2_costmap_2dは地図が完全に空(width/height=0)だと経路計画ができず、経路が無いと
ロボットは動けず、動けないとslam_toolboxの地図も育たない、という鶏と卵のデッドロックが
あるため、起動直後にこのノードが最初の地図ができるまで待つ。

以前は「このロボットのLiDARは1スキャンで360度全方位をカバーするため、静止した
ままの1回のスキャンでも旋回時と同じ範囲が既にobstacle_layerで埋まっており、
旋回の追加効果はほぼ無い」と判断して旋回を無効化していたが、これはローカルの
障害物回避(obstacle_layer)の話であり、explore_liteが見るグローバル地図
(slam_toolboxの/map)には当てはまらなかった。1点に静止したままだと壁の向こう側等
視線が通らない領域は既知にならず、既知の自由空間がスポーン地点周辺のごく狭い範囲に
留まってmin_frontier_sizeを満たすフロンティアがほぼ無い状態になり、explore_liteが
実際には未探索の地図に対して即座に「探索完了」と判定してしまう不具合を引き起こした。
そのため旋回を再度有効化している。

slam_toolboxはminimum_travel_distance/headingの制約により、初回スキャン以降は
ロボットが実際に動かない限り新しいスキャンを地図に取り込まない
(mapper_params_online_sync.yaml参照)。そのため「回るのをやめて別の合図を待つ」
という変更では解決しない。一方で、初回スキャンの時点(スポーン直後、まだ回転していない)
で既に自由空間が十分な場合はそれ以上回る必要が無いため、/mapの既知自由空間の面積が
一定値(MIN_FREE_AREA_M2)を超えた時点で旋回を打ち切る。360度回りきっても
満たさない場合は、従来通りの固定時間(SEED_DURATION_SEC)で終了する。
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker

ANGULAR_Z_RAD_S = 0.4
SEED_DURATION_SEC = 2 * math.pi / ANGULAR_Z_RAD_S  # ちょうど360度分(早期終了できない場合の上限)
PUBLISH_PERIOD_SEC = 0.1
MAP_READY_TIMEOUT_SEC = 20.0
STATUS_MARKER_ID = 0
# 半径1m相当の円が既知自由空間になれば、min_frontier_size(0.75m、explore_lite側の設定)を
# 満たすフロンティアが見つかる可能性が高いという経験則に基づく閾値。
MIN_FREE_AREA_M2 = 3.0
# occupancy gridの値(0=完全に自由〜100=完全に占有、-1=未知)のうち、自由とみなす閾値。
# nav2/map_serverの一般的な占有判定しきい値(50)に合わせる。
FREE_VALUE_THRESHOLD = 50


class InitialMapSeeder(Node):

    def __init__(self):
        super().__init__('initial_map_seeder')
        self._pub = self.create_publisher(TwistStamped, 'cmd_vel_nav2', 10)
        # RVizで「マッピング中」かどうかを見えるようにする、ロボット追従のテキストマーカー
        self._marker_pub = self.create_publisher(Marker, 'mapping_status', 10)
        # slam_toolboxのmapはtransient_local/reliableでpublishされるため、それに合わせる
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_ready = False
        self._sufficient_free_space = False
        self.create_subscription(OccupancyGrid, 'map', self._on_map, map_qos)
        # 完了をexplore_gate.pyに知らせ、旋回が終わるまでexplore_liteが動き出さない
        # ようにする(costmapが少しでもできた時点でexplore_liteは動けてしまうため)。
        done_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._done_pub = self.create_publisher(Empty, 'initial_map_seed_done', done_qos)
        # コンストラクタ時点では/clockをまだ一度も受信しておらずnow()が0を返すことがあり、
        # それを基準にすると次のタイマー周期で経過時間が一気に跳ね上がってしまうため、
        # 基準時刻は最初のタイマー発火時に確定させる
        self._start_time = None
        # 旋回そのものの開始時刻(map待ち完了後に確定する。Noneの間はまだ待機中)
        self._rotation_start_time = None
        self._rotation_start_tick = None
        # world_supervisor.pyがgzserverクラッシュ時に世界を再起動するとsim timeが0から
        # 巻き戻り、時刻基準の経過時間計算が負になって永久に完了しなくなる
        # (実際に発生し、cmd_vel_nav2を出し続けてNav2と競合し続けた)。
        # クロックの巻き戻りに影響されないtick数ベースでも経過時間を追跡する。
        self._tick_count = 0
        self._timer = self.create_timer(PUBLISH_PERIOD_SEC, self._on_timer)

    def _on_map(self, msg):
        if not self._map_ready:
            self._map_ready = True
            self.get_logger().info('slam_toolboxの初回mapを検知しました')
        if not self._sufficient_free_space and self._free_area_m2(msg) >= MIN_FREE_AREA_M2:
            self._sufficient_free_space = True
            self.get_logger().info(f'自由空間が{MIN_FREE_AREA_M2}m^2以上になりました')

    @staticmethod
    def _free_area_m2(grid_msg):
        resolution = grid_msg.info.resolution
        free_cells = sum(1 for v in grid_msg.data if 0 <= v < FREE_VALUE_THRESHOLD)
        return free_cells * resolution * resolution

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

        self._tick_count += 1

        if self._rotation_start_time is None:
            wait_elapsed_sec = (now - self._start_time).nanoseconds / 1e9
            wait_tick_elapsed_sec = self._tick_count * PUBLISH_PERIOD_SEC
            timed_out = (
                wait_elapsed_sec >= MAP_READY_TIMEOUT_SEC or
                wait_tick_elapsed_sec >= MAP_READY_TIMEOUT_SEC
            )
            if not self._map_ready and not timed_out:
                return
            if not self._map_ready:
                self.get_logger().warning(
                    f'{MAP_READY_TIMEOUT_SEC}秒待っても初回mapを検知できなかったため、'
                    '待たずに終了します')
            if self._sufficient_free_space:
                # 初回スキャンの時点で既に自由空間が十分(開けた場所にスポーンした場合)。
                # 1度も回転せずに終了する。
                self._finish_seeding()
                return
            self._rotation_start_time = now
            self._rotation_start_tick = self._tick_count
            return

        # --- その場360度旋回(自由空間が十分になり次第、途中で打ち切る) ---
        elapsed_sec = (now - self._rotation_start_time).nanoseconds / 1e9
        tick_elapsed_sec = (self._tick_count - self._rotation_start_tick) * PUBLISH_PERIOD_SEC
        reached_limit = elapsed_sec >= SEED_DURATION_SEC or tick_elapsed_sec >= SEED_DURATION_SEC
        if reached_limit or self._sufficient_free_space:
            self._finish_seeding()
            return

        cmd = TwistStamped()
        cmd.header.stamp = now.to_msg()
        cmd.twist.angular.z = ANGULAR_Z_RAD_S
        self._pub.publish(cmd)
        self._publish_status_marker(active=True)

    def _finish_seeding(self):
        self._pub.publish(TwistStamped())
        self._publish_status_marker(active=False)
        self._done_pub.publish(Empty())
        self.get_logger().info('初期地図の自動生成を終了しました')
        self._timer.cancel()


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
