#!/usr/bin/env python3
"""指定エンティティを、物理エンジンを一時停止した状態で削除する。

Gazebo Classicはセンサープラグイン付きモデルの削除でgzserver自体がsegfaultすることがある
既知の問題があり(upstream、EOLのため修正見込みなし)、削除前後にpause_physics/unpause_physicsを
挟むことで頻度を下げる。`ros2 service call`を3回シェルアウトする実装は、呼び出しごとに
新しいDDSコンテキストの探索が必要になり低速・不安定だったため、単一のrclpyコンテキストで
タイムアウト付きに完結させる(entity_reaper.pyと同じ方式)。
"""
import sys

import rclpy
from gazebo_msgs.srv import DeleteEntity
from std_srvs.srv import Empty

SERVICE_TIMEOUT_SEC = 5.0


def _call(node, client, request, service_name):
    if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_SEC):
        node.get_logger().warn(f'{service_name}が利用できません。スキップします')
        return
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=SERVICE_TIMEOUT_SEC)


def main():
    entity_name = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node('delete_entity_safely')

    pause_client = node.create_client(Empty, '/pause_physics')
    delete_client = node.create_client(DeleteEntity, '/delete_entity')
    unpause_client = node.create_client(Empty, '/unpause_physics')

    _call(node, pause_client, Empty.Request(), '/pause_physics')
    _call(node, delete_client, DeleteEntity.Request(name=entity_name), '/delete_entity')
    _call(node, unpause_client, Empty.Request(), '/unpause_physics')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
