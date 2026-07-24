#!/usr/bin/env python3
"""spawn_robotが生成したエンティティのうち、対応するrobot_state_publisherノードが
存在しないものをGazeboから削除する常駐監視ノード。

spawn_robot.launch.pyはlaunch終了時にOnShutdownで自分自身のエンティティを削除するが、
VS Codeのnode-terminal停止(特にコンテナ内開発時)はSIGKILLでプロセスツリーを丸ごと
終了させることがあり、その場合はどのプロセスも後片付けコードを実行できない。
このノードはspawn_robotのプロセスツリーの外側で独立して動くため、その種のkillの
され方に影響されない。
"""
import re

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import DeleteEntity, GetModelList
from std_srvs.srv import Empty

ENTITY_PATTERN = re.compile(r'^tb3_.+$')
CHECK_PERIOD_SEC = 5.0
# spawn直後の一瞬(エンティティはあるがノードがまだ上がっていない)を誤って
# 削除しないよう、この回数連続で不在が続いたときだけ削除する
MISSING_THRESHOLD = 2


class EntityReaper(Node):

    def __init__(self):
        super().__init__('entity_reaper')
        self._get_models = self.create_client(GetModelList, '/get_model_list')
        self._delete_entity = self.create_client(DeleteEntity, '/delete_entity')
        self._pause_physics = self.create_client(Empty, '/pause_physics')
        self._unpause_physics = self.create_client(Empty, '/unpause_physics')
        self._missing_counts = {}
        self.create_timer(CHECK_PERIOD_SEC, self._check)

    def _check(self):
        if not self._get_models.service_is_ready():
            return
        future = self._get_models.call_async(GetModelList.Request())
        future.add_done_callback(self._on_model_list)

    def _on_model_list(self, future):
        try:
            model_names = future.result().model_names
        except Exception as exc:
            self.get_logger().warn(f'/get_model_list呼び出しに失敗: {exc}')
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
        # Gazebo Classicはセンサープラグイン(camera/lidar等)付きモデルの削除でgzserver自体が
        # segfaultすることがある既知の問題があるため、削除前に物理エンジンを一時停止する
        future = self._pause_physics.call_async(Empty.Request())
        future.add_done_callback(lambda f, e=entity: self._on_paused(f, e))

    def _on_paused(self, future, entity):
        try:
            future.result()
        except Exception as exc:
            self.get_logger().warn(f'{entity}: pause_physics呼び出しでエラー: {exc}')
        future = self._delete_entity.call_async(DeleteEntity.Request(name=entity))
        future.add_done_callback(lambda f, e=entity: self._on_reap_done(f, e))

    def _on_reap_done(self, future, entity):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f'{entity}: 削除しました')
            else:
                self.get_logger().warn(f'{entity}: 削除に失敗: {result.status_message}')
        except Exception as exc:
            self.get_logger().warn(f'{entity}: 削除呼び出しでエラー: {exc}')
        finally:
            self._unpause_physics.call_async(Empty.Request())


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
