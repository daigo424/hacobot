#!/usr/bin/env python3
"""create_world.launch.pyを監視し、クラッシュしたらワールド全体を自動再起動する。

Gazebo Classic時代はセンサープラグイン付きモデルの削除でgzserver自体がsegfaultする
既知のバグがあった(upstream、EOLのため修正見込みなし)。Harmonic(gz sim)で同種の問題が
再現するかは未確認だが、クラッシュ自体は環境要因(WSL2のホストフリーズ等)でも起こり得るため、
クラッシュのたびに手動でCreate Worldを再起動する手間を無くす常駐ラッパーとして維持する。
再起動するとその時点の全ロボットは失われる。
"""
import os
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


def _gz_sim_server_alive():
    return subprocess.run(
        ['pgrep', '-f', '^gz sim server$'], stdout=subprocess.DEVNULL
    ).returncode == 0


def _wait_for_crash_or_exit(proc):
    # gz simがsegfaultしても`ros2 launch`本体はserver_required指定なしでは
    # 終了しないため、proc.poll()だけでは検知できない。gz sim server自体の生死を見る。
    start_time = time.time()
    gz_sim_server_seen = False
    while not _shutdown_requested:
        if _gz_sim_server_alive():
            gz_sim_server_seen = True
        elif gz_sim_server_seen:
            print('[world_supervisor] gz sim serverが停止しました', flush=True)
            return
        elif time.time() - start_time > STARTUP_GRACE_SEC:
            print('[world_supervisor] gz sim serverが起動猶予内に起動しませんでした', flush=True)
            return

        if proc.poll() is not None:
            print(f'[world_supervisor] create_world.launch.pyが終了しました(exit={proc.returncode})', flush=True)
            return

        time.sleep(POLL_INTERVAL_SEC)


def main():
    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    while not _shutdown_requested:
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
