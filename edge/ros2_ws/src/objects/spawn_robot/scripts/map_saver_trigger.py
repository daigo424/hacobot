#!/usr/bin/env python3
"""explore_liteの探索完了(EXPLORATION_COMPLETE)、または手動トリガーを検知したら、地図を保存する。

explore_lite自体には地図の保存機能が無いため、/explore/statusを監視し、
探査完了を検知した時点でnav2_map_serverのmap_saver_cliを呼び出す。
手動操縦(build_map.launch.pyのexploration_mode:=manual)ではexplore_liteが
動かないため、代わりにsave_map_triggerトピックへのpublishで同じ保存処理を起動できる。

保存成功時、SLAM開始点(Gazebo世界座標)を*.origin.yamlへ記録する。spawn_robot.launch.py
のAMCLモードが、この地図を使う後続のスポーンでmapフレームの初期位置を自動算出するために使う。
"""
import os
import subprocess
import sys
import time

import rclpy
import yaml
from explore_lite_msgs.msg import ExploreStatus
from rclpy.node import Node
from std_msgs.msg import Empty

SAVE_TIMEOUT_SEC = 15.0


class MapSaverTrigger(Node):

    def __init__(self, map_path, origin_x, origin_y):
        super().__init__('map_saver_trigger')
        self._map_path = map_path
        self._origin_x = origin_x
        self._origin_y = origin_y
        self._saved = False
        self.create_subscription(
            ExploreStatus, 'explore/status', self._on_explore_status, 10)
        self.create_subscription(
            Empty, 'save_map_trigger', self._on_manual_trigger, 10)

    def _backup_existing_map(self):
        # 削除ではなくタイムスタンプ付きでリネーム退避する(探査が失敗しても地図が
        # 1つも無くなる事態を避けるため。保存成功時はどのみち新しい地図で上書きされる)。
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        for suffix in ('.yaml', '.pgm', '.origin.yaml'):
            src = f'{self._map_path}{suffix}'
            if os.path.exists(src):
                backup = f'{src}.{timestamp}.bak'
                os.rename(src, backup)
                self.get_logger().info(f'既存の地図を退避しました: {backup}')

    def _on_explore_status(self, msg):
        if msg.status != ExploreStatus.EXPLORATION_COMPLETE:
            return
        self._save_map('探索完了を検知しました。')

    def _on_manual_trigger(self, _msg):
        self._save_map('手動保存トリガーを検知しました。')

    def _save_map(self, reason):
        if self._saved:
            return
        self._saved = True
        self.get_logger().info(f'{reason}地図を保存します: {self._map_path}')

        os.makedirs(os.path.dirname(self._map_path), exist_ok=True)
        self._backup_existing_map()
        ns = self.get_namespace()
        try:
            result = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', self._map_path,
                 '--ros-args', '-r', f'__ns:={ns}',
                 # 既定ノード名'map_saver'はSLAMモードのmap_saverノードと同一namespace内で衝突し、
                 # "Waiting on external lifecycle transitions to activate"のまま失敗するため改名する。
                 '-r', '__node:=map_saver_cli_trigger',
                 # save_map_timeoutの既定値(2.0秒)だとCPU負荷でslam_toolboxの/map配信が
                 # 一時的に遅れた際に"Failed to spin map subscription"で保存自体が失敗する。
                 '-p', 'save_map_timeout:=10.0'],
                capture_output=True, text=True, timeout=SAVE_TIMEOUT_SEC,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().error('地図の保存がタイムアウトしました')
            return

        if result.returncode == 0:
            self._write_origin_file()
            self.get_logger().info(f'地図を保存しました: {self._map_path}.yaml')
            self._show_dialog('地図生成 完了', f'地図を保存しました:\n{self._map_path}.yaml')
        else:
            self.get_logger().error(f'地図の保存に失敗しました: {result.stderr.strip()}')
            self._show_dialog('地図生成 失敗', f'地図の保存に失敗しました:\n{result.stderr.strip()}')

    def _write_origin_file(self):
        origin_path = f'{self._map_path}.origin.yaml'
        with open(origin_path, 'w') as f:
            yaml.safe_dump({'x': float(self._origin_x), 'y': float(self._origin_y)}, f)

    def _show_dialog(self, title, message):
        # rclpy.spin()をブロックしないよう別プロセスで表示し、閉じ忘れてもノード終了は妨げない
        script = (
            'import tkinter as tk\n'
            'from tkinter import messagebox\n'
            'root = tk.Tk()\n'
            'root.withdraw()\n'
            f'messagebox.showinfo({title!r}, {message!r})\n'
        )
        subprocess.Popen([sys.executable, '-c', script])


def main():
    rclpy.init()
    node = MapSaverTrigger(sys.argv[1], sys.argv[2], sys.argv[3])
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
