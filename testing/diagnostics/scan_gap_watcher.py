#!/usr/bin/env python3
"""センサートピックの受信間隔を監視し、閾値を超えた途絶(gap)を検知した瞬間を記録する診断スクリプト。

watchdogノードとは独立した観測点でトピックの途絶を記録することで、他の監視ログ
(例: pidstatのサンプリング欠落)と時刻を突き合わせ、原因がアプリ層(gzserver/DDS)なのか
ホスト層(WSL2 VMのフリーズ等)なのかを切り分けるのに使う。
2026-07-21、WSL2ホストの周期的フリーズが`/scan`途絶の原因だと特定した調査で使用した
(詳細はREADME「既知の制約」参照)。

実行例(ros2_nav2_containerの中で):
    python3 testing/diagnostics/scan_gap_watcher.py \\
        --topic /scan@sensor_msgs/msg/LaserScan \\
        --topic /camera/image_raw@sensor_msgs/msg/Image \\
        --threshold-ms 300
"""
from __future__ import annotations

import argparse
import importlib
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


@dataclass
class MonitoredTopic:
    name: str
    type_str: str


def parse_topic_arg(raw: str) -> MonitoredTopic:
    name, sep, type_str = raw.partition("@")
    if not sep:
        raise argparse.ArgumentTypeError(
            f"'{raw}' は 'トピック名@パッケージ/msg/型名' の形式で指定してください"
            "(例: /scan@sensor_msgs/msg/LaserScan)")
    return MonitoredTopic(name=name, type_str=type_str)


def resolve_message_class(type_str: str):
    package, _, message = type_str.partition("/msg/")
    module = importlib.import_module(f"{package}.msg")
    return getattr(module, message)


class GapWatcher(Node):
    def __init__(self, topics: list[MonitoredTopic], threshold_ms: float):
        super().__init__("scan_gap_watcher")
        self._threshold_s = threshold_ms / 1000.0
        self._last_seen: dict[str, float] = {}
        self._subs = []
        for topic in topics:
            msg_class = resolve_message_class(topic.type_str)
            sub = self.create_subscription(
                msg_class, topic.name,
                self._make_callback(topic.name), qos_profile_sensor_data)
            self._subs.append(sub)
            self.get_logger().info(f"Watching {topic.name} ({topic.type_str})")

    def _make_callback(self, topic_name: str):
        def callback(_msg) -> None:
            now = time.time()
            last = self._last_seen.get(topic_name)
            if last is not None:
                gap_s = now - last
                if gap_s > self._threshold_s:
                    print(f"{now:.3f} GAP {topic_name} {gap_s * 1000:.0f}ms", flush=True)
            self._last_seen[topic_name] = now
        return callback


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--topic", dest="topics", action="append", required=True, type=parse_topic_arg,
        help="監視するトピック('トピック名@パッケージ/msg/型名'形式、複数指定可)")
    parser.add_argument(
        "--threshold-ms", type=float, default=300.0,
        help="この間隔(ms)を超えたらGAPとして出力する(既定: 300ms)")
    args = parser.parse_args()

    rclpy.init()
    node = GapWatcher(args.topics, args.threshold_ms)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
