#!/usr/bin/env python3
"""slam_toolboxをconfigure→activateする。

nav2_bringupのslam_launch.pyは、included先のslam_toolbox付属online_sync_launch.py
が持つ自動configure/activateの仕組み(autostart)に頼っているが、autostart引数自体が
slam_launch.pyからonline_sync_launch.pyへ渡されておらず(upstream側の抜け漏れ、
jazzyブランチでも同様)、かつonline_sync_launch.py自身のデフォルトtrueによる
自動発火も実測では起きないことが多い(手動でros2 lifecycle setすると即座に
成功する)。そのため、このノード自身でconfigure/activateを明示的に呼ぶ。
"""
import subprocess
import sys
import time

RETRY_INTERVAL_SEC = 1.0
MAX_ATTEMPTS = 60
SUBPROCESS_TIMEOUT_SEC = 10.0


def _lifecycle_set(node_name, transition):
    try:
        return subprocess.run(
            ['ros2', 'lifecycle', 'set', node_name, transition],
            capture_output=True, text=True, timeout=SUBPROCESS_TIMEOUT_SEC,
        )
    except subprocess.TimeoutExpired:
        # 高負荷時はros2 CLI自体の応答が遅れることがあるが、これは単なる
        # このattemptの失敗として扱い、リトライループに委ねる
        return None


def _wait_for_transition(node_name, transition):
    for _ in range(MAX_ATTEMPTS):
        result = _lifecycle_set(node_name, transition)
        if result is not None and result.returncode == 0:
            return True
        time.sleep(RETRY_INTERVAL_SEC)
    return False


def main():
    node_name = sys.argv[1]

    if not _wait_for_transition(node_name, 'configure'):
        print(f'{node_name}: configureに失敗しました', file=sys.stderr)
        return
    if not _wait_for_transition(node_name, 'activate'):
        print(f'{node_name}: activateに失敗しました', file=sys.stderr)


if __name__ == '__main__':
    main()
