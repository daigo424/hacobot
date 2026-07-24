#!/usr/bin/env python3
"""spawn_robotが生成したエンティティのうち、対応するrobot_state_publisherノードが
存在しないものをGazebo(gz sim)から削除する常駐監視ノード。

spawn_robot.launch.pyはlaunch終了時にOnShutdownで自分自身のエンティティを削除するが、
VS Codeのnode-terminal停止(特にコンテナ内開発時)はSIGKILLでプロセスツリーを丸ごと
終了させることがあり、その場合はどのプロセスも後片付けコードを実行できない。
このノードはspawn_robotのプロセスツリーの外側で独立して動くため、その種のkillの
され方に影響されない。

Gazebo Harmonicにはgazebo_msgs/srv/GetModelListに相当する常駐ROS2サービスが標準では
無いため、`gz model --list`(CLI)を直接叩いてエンティティ一覧を取得する。削除も同様に
`ros2 run ros_gz_sim remove`をサブプロセスとして呼ぶ。
"""
import re
import subprocess

import rclpy
from rclpy.node import Node

ENTITY_PATTERN = re.compile(r'^tb3_.+$')
CHECK_PERIOD_SEC = 5.0
# spawn直後の一瞬(エンティティはあるがノードがまだ上がっていない)を誤って
# 削除しないよう、この回数連続で不在が続いたときだけ削除する
MISSING_THRESHOLD = 2
SUBPROCESS_TIMEOUT_SEC = 5.0


class EntityReaper(Node):

    def __init__(self):
        super().__init__('entity_reaper')
        self._missing_counts = {}
        self.create_timer(CHECK_PERIOD_SEC, self._check)

    def _list_entities(self):
        try:
            result = subprocess.run(
                ['gz', 'model', '--list'],
                capture_output=True, text=True, timeout=SUBPROCESS_TIMEOUT_SEC,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().warn('gz model --listがタイムアウトしました')
            return None
        if result.returncode != 0:
            self.get_logger().warn(f'gz model --listに失敗: {result.stderr.strip()}')
            return None
        return {
            line.strip().lstrip('- ').strip()
            for line in result.stdout.splitlines()
            if line.strip().startswith('-')
        }

    def _check(self):
        model_names = self._list_entities()
        if model_names is None:
            return

        live_pairs = set(self.get_node_names_and_namespaces())
        entities = {name for name in model_names if ENTITY_PATTERN.match(name)}

        for entity in entities:
            if ('robot_state_publisher', f'/{entity}') in live_pairs:
                self._missing_counts.pop(entity, None)
                continue
            count = self._missing_counts.get(entity, 0) + 1
            self._missing_counts[entity] = count
            if count >= MISSING_THRESHOLD:
                self._reap(entity)
                self._missing_counts.pop(entity, None)

        for stale in set(self._missing_counts) - entities:
            self._missing_counts.pop(stale, None)

    def _reap(self, entity):
        self.get_logger().warn(f'{entity}: 対応するノードが無いため削除します')
        try:
            result = subprocess.run(
                ['ros2', 'run', 'ros_gz_sim', 'remove', '--ros-args', '-p',
                 f'entity_name:={entity}'],
                capture_output=True, text=True, timeout=SUBPROCESS_TIMEOUT_SEC,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f'{entity}: 削除呼び出しがタイムアウトしました')
            return
        if 'successful' in result.stdout.lower():
            self.get_logger().info(f'{entity}: 削除しました')
        else:
            self.get_logger().warn(f'{entity}: 削除に失敗: {result.stdout.strip()}')


def main():
    rclpy.init()
    node = EntityReaper()
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
