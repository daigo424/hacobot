#!/usr/bin/env python3
"""create_world.launch.pyを監視し、クラッシュしたらワールド全体を自動再起動する。

Gazebo Classicはセンサープラグイン付きモデルの削除でgzserver自体がsegfaultすることがある
既知のバグがあり(upstream、EOLのため修正見込みなし)、pause_physics緩和策でも完全には
防げない。クラッシュのたびに手動でCreate Worldを再起動する手間を無くすための常駐ラッパー。
再起動するとその時点の全ロボットは失われる。
"""
import os
import shutil
import signal
import subprocess
import time

POLL_INTERVAL_SEC = 2.0
STARTUP_GRACE_SEC = 20.0
RESTART_DELAY_SEC = 2.0
_shutdown_requested = False


def _handle_signal(signum, frame):
    global _shutdown_requested
    _shutdown_requested = True


def _terminate_group(pgid, sig):
    try:
        os.killpg(pgid, sig)
    except ProcessLookupError:
        pass


def _clear_stale_gazebo_state():
    # 強制終了の繰り返しで~/.gazebo/{server,client}-11345が壊れ、次回起動がexit 255で
    # 即死することがある(2026-07-24に確認済み)。再起動ループがこれで詰まらないよう毎回消す。
    for name in ('server-11345', 'client-11345'):
        shutil.rmtree(os.path.expanduser(f'~/.gazebo/{name}'), ignore_errors=True)


def _gzserver_alive():
    return subprocess.run(
        ['pgrep', '-f', '^gzserver '], stdout=subprocess.DEVNULL
    ).returncode == 0


def _wait_for_crash_or_exit(proc):
    # gzserverがsegfaultしても`ros2 launch`本体はserver_required指定なしでは
    # 終了しないため、proc.poll()だけでは検知できない。gzserver自体の生死を見る。
    start_time = time.time()
    gzserver_seen = False
    while not _shutdown_requested:
        if _gzserver_alive():
            gzserver_seen = True
        elif gzserver_seen:
            print('[world_supervisor] gzserverが停止しました', flush=True)
            return
        elif time.time() - start_time > STARTUP_GRACE_SEC:
            print('[world_supervisor] gzserverが起動猶予内に起動しませんでした', flush=True)
            return

        if proc.poll() is not None:
            print(f'[world_supervisor] create_world.launch.pyが終了しました(exit={proc.returncode})', flush=True)
            return

        time.sleep(POLL_INTERVAL_SEC)


def main():
    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    while not _shutdown_requested:
        _clear_stale_gazebo_state()
        print('[world_supervisor] create_world.launch.pyを起動します', flush=True)
        proc = subprocess.Popen(
            ['ros2', 'launch', 'create_world', 'create_world.launch.py'],
            start_new_session=True,
        )
        pgid = proc.pid

        _wait_for_crash_or_exit(proc)

        if _shutdown_requested:
            _terminate_group(pgid, signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                pass
            _terminate_group(pgid, signal.SIGKILL)
            break

        print('[world_supervisor] ワールドを再起動します', flush=True)
        _terminate_group(pgid, signal.SIGKILL)
        time.sleep(RESTART_DELAY_SEC)


if __name__ == '__main__':
    main()
