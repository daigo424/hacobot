#!/usr/bin/env python3
"""指定エンティティをGazebo(gz sim)から削除する。

`ros2 run ros_gz_sim remove`を直接シェルアウトする代わりにこのスクリプトを経由するのは、
将来Gazebo Harmonic固有の削除時クラッシュ等が見つかった場合の緩和策を1箇所に集約するため
(Gazebo Classicではセンサープラグイン付きモデルの削除でgzserver自体がsegfaultする既知の
問題があり、pause_physics/unpause_physicsで挟む緩和策が必要だった。Harmonicで同種の問題が
再現するかは未確認のため、現時点では単純な削除のみ行う)。
"""
import subprocess
import sys

TIMEOUT_SEC = 10.0


def main():
    entity_name = sys.argv[1]
    subprocess.run(
        ['ros2', 'run', 'ros_gz_sim', 'remove', '--ros-args', '-p',
         f'entity_name:={entity_name}'],
        timeout=TIMEOUT_SEC,
    )


if __name__ == '__main__':
    main()
