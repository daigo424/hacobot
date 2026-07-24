#!/usr/bin/env python3
"""hacobot 耐久試験(Soak Test)自動実行スクリプト。

Gazebo上でロボットを指定時間、複数のウェイポイントを巡回させ続けながら、
メモリ使用量・再起動回数・安全状態遷移履歴・Nav2応答時間を5分間隔(既定)で記録し、
合格基準を最後に自動判定してJSONレポートに出力する。

前提: このスクリプトは ros2_nav2_container 内で、Gazebo+Nav2+safety_bringup
(edge/ros2_ws/src/nav2_bringup_custom/launch/nav2_bringup.launch.py)が
既に起動している状態で実行する。

実行例:
    docker exec ros2_nav2_container bash -c \\
        "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \\
         python3 /workspace/src/../../../testing/soak_test/run_soak_test.py --duration 1h"

    # もしくはMakefile経由 (下記参照)
    $ python testing/soak_test/run_soak_test.py --duration 1h   # 短時間ドライラン
    $ python testing/soak_test/run_soak_test.py --duration 24h  # 標準耐久試験
    $ python testing/soak_test/run_soak_test.py --duration 72h  # 拡張耐久試験(合格判定用)
"""
from __future__ import annotations

import argparse
import json
import re
import statistics
import subprocess
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional

import psutil
import rclpy

# ロジック名 -> プロセスのコマンドライン中に含まれうる部分文字列群(いずれかに一致すればOK)。
# component_container_isolated はNav2の複数ノード(bt_navigator, controller_server等)を
# 1プロセスにまとめて動かすコンテナプロセスのため、Nav2全体をまとめて1エントリとして扱う。
# gz_sim_serverが2パターンあるのは、起動経路によってGazebo(Harmonic)のサーバープロセスの
# コマンドラインが変わるため: nav2_bringup_custom/gazebo_sim.launch.py(turtlebot3本家の
# turtlebot3_world.launch.py)は"-s"付きで直接起動するため"gz sim -r -s"、spawn_robotの
# create_world.launch.pyは分割せず起動するため内部で"gz sim server"という名前の子プロセスに
# なる。どちらか一方だけだと使う起動経路によってはGazeboの負荷が一切記録されない。
MONITORED_PROCESSES: dict[str, tuple[str, ...]] = {
    "heartbeat_monitor": ("heartbeat_monitor_node",),
    "safety_state_machine": ("safety_state_machine_node",),
    "watchdog": ("watchdog_node",),
    "estop_bridge": ("estop_bridge_node",),
    "nav2_heartbeat_adapter": ("nav2_heartbeat_adapter_node",),
    "nav2_stack": ("component_container_isolated",),
    "gz_sim_server": ("gz sim -r -s", "gz sim server"),
}

# turtlebot3_world内の、costmap生成後であれば通常到達可能な小さな周回コース。
DEFAULT_WAYPOINTS = [
    (0.5, 0.0),
    (0.5, 0.5),
    (0.0, 0.5),
    (0.0, 0.0),
]


def parse_duration(text: str) -> float:
    """"1h" "30m" "90s" "24h" のような文字列を秒数(float)に変換する。"""
    m = re.fullmatch(r"(\d+(?:\.\d+)?)\s*([hms])", text.strip().lower())
    if not m:
        raise argparse.ArgumentTypeError(
            f"duration must look like '1h', '30m', '90s' (got: {text!r})")
    value, unit = float(m.group(1)), m.group(2)
    return value * {"h": 3600.0, "m": 60.0, "s": 1.0}[unit]


@dataclass
class ProcessSample:
    found: bool
    pid: Optional[int] = None
    rss_bytes: int = 0


def sample_process(match_substrings: tuple[str, ...]) -> ProcessSample:
    """コマンドラインにmatch_substringsのいずれかを含む全プロセスのRSSを合算してサンプリングする。"""
    total_rss = 0
    first_pid: Optional[int] = None
    found = False
    for proc in psutil.process_iter(attrs=["pid", "cmdline"]):
        try:
            cmdline = " ".join(proc.info["cmdline"] or [])
            if any(s in cmdline for s in match_substrings):
                found = True
                if first_pid is None:
                    first_pid = proc.info["pid"]
                total_rss += proc.memory_info().rss
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return ProcessSample(found=found, pid=first_pid, rss_bytes=total_rss)


class NodeProcessTracker:
    """監視対象プロセス群のメモリ使用量推移と、PID変化による再起動回数を追跡する。"""

    def __init__(self, targets: dict[str, tuple[str, ...]]):
        self._targets = targets
        self._last_pid: dict[str, Optional[int]] = {name: None for name in targets}
        self._restart_counts: dict[str, int] = {name: 0 for name in targets}
        self._history: dict[str, list[dict]] = {name: [] for name in targets}

    def sample(self, elapsed_sec: float) -> None:
        for logical_name, match_substrings in self._targets.items():
            sample = sample_process(match_substrings)
            if sample.found and sample.pid is not None:
                last_pid = self._last_pid[logical_name]
                if last_pid is not None and sample.pid != last_pid:
                    self._restart_counts[logical_name] += 1
                self._last_pid[logical_name] = sample.pid
            self._history[logical_name].append({
                "elapsed_sec": round(elapsed_sec, 1),
                "found": sample.found,
                "rss_bytes": sample.rss_bytes,
            })

    def summary(self) -> dict:
        result = {}
        for logical_name in self._targets:
            samples = [s["rss_bytes"] for s in self._history[logical_name] if s["found"]]
            first_rss = samples[0] if samples else None
            last_rss = samples[-1] if samples else None
            increase_pct = None
            if first_rss and last_rss is not None and first_rss > 0:
                increase_pct = (last_rss - first_rss) / first_rss * 100.0
            result[logical_name] = {
                "restart_count": self._restart_counts[logical_name],
                "first_rss_bytes": first_rss,
                "last_rss_bytes": last_rss,
                "memory_increase_pct": increase_pct,
                "samples": self._history[logical_name],
            }
        return result


class SafetyStateWatcher:
    """/safety/state をポーリングし、状態遷移履歴を記録する。

    当初はrclpyのSubscriptionでイベント駆動に監視する設計にしていたが、
    Nav2巡回(run_nav2_patrol)用に別スレッドでrclpyコンテキストを使っている状態で
    このノード用にさらに別コンテキスト/Executorを同時に使うと、
    `IndexError: wait set index too big` や最終的にはセグメンテーション違反が
    実際に発生した(rclpyは同一プロセス内での複数コンテキストの並行spinを
    安全に扱えないことがある、という実地で踏んだ制約)。
    そのため、rclpyを直接使わず`ros2 topic echo --once`をサブプロセスとして
    都度起動するポーリング方式に変更し、rclpyの使用をrun_nav2_patrolスレッド1箇所に
    限定した。取得粒度はsample_interval_sec(既定5分)単位になるが、
    本試験の目的(長時間の安定性確認)には十分。
    """

    def __init__(self):
        self.history: list[dict] = []
        self._last_state: Optional[str] = None

    def poll(self, elapsed_sec: float) -> None:
        state = self._echo_once()
        if state is None or state == self._last_state:
            return
        self._last_state = state
        self.history.append({"elapsed_sec": round(elapsed_sec, 1), "state": state})

    @staticmethod
    def _echo_once() -> Optional[str]:
        try:
            result = subprocess.run(
                ["timeout", "3", "ros2", "topic", "echo", "/safety/state",
                 "--once", "--field", "data"],
                capture_output=True, text=True, timeout=5,
            )
        except (subprocess.TimeoutExpired, FileNotFoundError):
            return None
        # `--field`指定時も出力末尾にYAMLドキュメント区切り"---"が付与されるため、
        # 1行目(実際の値)だけを取り出す
        first_line = result.stdout.strip().splitlines()[0].strip() if result.stdout.strip() else ""
        return first_line if first_line else None


@dataclass
class Nav2ResponseTimeTracker:
    """5分ウィンドウ毎のNav2ゴール応答時間平均を集計する。"""

    window_sec: float
    _window_start: float = field(default_factory=time.monotonic)
    _current_window: list[float] = field(default_factory=list)
    windows: list[dict] = field(default_factory=list)

    def record(self, elapsed_sec: float, response_time_sec: float) -> None:
        now = time.monotonic()
        if now - self._window_start >= self.window_sec and self._current_window:
            self._flush(elapsed_sec)
        self._current_window.append(response_time_sec)

    def _flush(self, elapsed_sec: float) -> None:
        if self._current_window:
            self.windows.append({
                "elapsed_sec": round(elapsed_sec, 1),
                "avg_response_time_sec": statistics.mean(self._current_window),
                "sample_count": len(self._current_window),
            })
        self._current_window = []
        self._window_start = time.monotonic()

    def finalize(self, elapsed_sec: float) -> None:
        self._flush(elapsed_sec)


def run_nav2_patrol(
    stop_event: threading.Event,
    tracker: Nav2ResponseTimeTracker,
    start_time: float,
    waypoints: list[tuple[float, float]],
) -> None:
    """BasicNavigatorでウェイポイントを巡回し続け、ゴール毎の応答時間を記録する。

    Nav2/nav2_simple_commanderが使えない環境(このスクリプト単体のユニットテスト等)でも
    Soak Test自体は継続できるよう、インポート・初期化失敗時は握りつぶしてログのみ出す。
    """
    try:
        from geometry_msgs.msg import PoseStamped
        from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    except ImportError as exc:
        print(f"[soak_test] nav2_simple_commander not available, skipping patrol: {exc}")
        return

    navigator = BasicNavigator()
    # waitUntilNav2Active()は既定でAMCLの/get_stateサービスを待つが、このプロジェクトは
    # AMCLではなくslam_toolbox(オンラインSLAM)を自己位置推定源として使っており、
    # かつこのビルドのslam_toolboxノードは標準lifecycle nodeの/get_stateサービスを
    # 実装していない(実際に'localizer=slam_toolbox'を指定して試したところ、
    # サービスが永遠に見つからずハングすることを確認した)。そのため、
    # bt_navigatorの活性化のみを待つ(内部で使われているprivateメソッドを直接呼ぶ)。
    # ゴールが早すぎるタイミングで送られて即座にrejectされ続ける事態
    # (最初の実装で実際に踏んだ不具合)を防ぐのが目的。
    navigator._waitForNodeToActivate("bt_navigator")

    idx = 0
    while not stop_event.is_set():
        x, y = waypoints[idx % len(waypoints)]
        idx += 1

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = navigator.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0

        goal_start = time.monotonic()
        navigator.goToPose(goal)
        while not navigator.isTaskComplete():
            if stop_event.is_set():
                navigator.cancelTask()
                return
            time.sleep(0.5)
        response_time = time.monotonic() - goal_start
        tracker.record(time.monotonic() - start_time, response_time)

        # ゴールが即座にrejectされる(応答時間がほぼ0)状態が続くと、対策なしでは
        # ここが実質的なビジーループになりCPU・ログを浪費する。成功可否によらず
        # 次のゴールまで最低限のクールダウンを入れてレートを制限する。
        # (rejectが続く場合はバックオフを伸ばし、無駄な再送を減らす)
        result = navigator.getResult()
        cooldown = 1.0 if result == TaskResult.SUCCEEDED else 3.0
        for _ in range(int(cooldown * 10)):
            if stop_event.is_set():
                return
            time.sleep(0.1)


def build_report(
    args: argparse.Namespace,
    process_summary: dict,
    safety_state_history: list[dict],
    nav2_windows: list[dict],
    actual_duration_sec: float,
) -> dict:
    memory_increases = [
        v["memory_increase_pct"] for v in process_summary.values()
        if v["memory_increase_pct"] is not None
    ]
    total_restarts = sum(v["restart_count"] for v in process_summary.values())

    # C-1: 72時間試験のみ厳密判定。それ未満の試験時間では参考値として記録する。
    c1_applicable = args.duration_sec >= 72 * 3600 * 0.95
    c1_pass = all(v <= 10.0 for v in memory_increases) if memory_increases else None

    c2_pass = (total_restarts == 0)

    c3_pass = None
    if len(nav2_windows) >= 2:
        first_avg = nav2_windows[0]["avg_response_time_sec"]
        last_avg = nav2_windows[-1]["avg_response_time_sec"]
        if first_avg > 0:
            c3_pass = last_avg <= first_avg * 1.5

    c4_pass = safety_state_history[-1]["state"] in ("NORMAL", "DEGRADED") \
        if safety_state_history else None

    return {
        "run_started_at": args.run_started_at,
        "requested_duration_sec": args.duration_sec,
        "actual_duration_sec": round(actual_duration_sec, 1),
        "sample_interval_sec": args.sample_interval_sec,
        "process_memory": process_summary,
        "safety_state_history": safety_state_history,
        "nav2_response_time_windows": nav2_windows,
        "pass_criteria": {
            "C-1_memory_increase_under_10pct": {
                "applicable_to_this_run": c1_applicable,
                "result": c1_pass,
                "detail": "72時間未満の試験では参考値",
            },
            "C-2_zero_unexpected_restarts": {
                "result": c2_pass,
                "total_restart_count": total_restarts,
            },
            "C-3_nav2_response_time_within_150pct": {
                "result": c3_pass,
            },
            "C-4_safety_state_recoverable": {
                "result": c4_pass,
                "final_state": safety_state_history[-1]["state"] if safety_state_history else None,
            },
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--duration", type=parse_duration, default=parse_duration("24h"),
        help="試験時間 (例: 1h, 24h, 72h)。既定値は24h")
    parser.add_argument(
        "--sample-interval", type=parse_duration, default=parse_duration("5m"),
        dest="sample_interval_sec",
        help="メトリクス記録間隔 (既定値は5m = 300秒)")
    parser.add_argument(
        "--output-dir", type=Path,
        default=Path(__file__).parent / "reports",
        help="レポート出力先の親ディレクトリ")
    args = parser.parse_args()
    args.duration_sec = args.duration

    run_started_at = datetime.now()
    args.run_started_at = run_started_at.isoformat()
    report_dir = args.output_dir / run_started_at.strftime("%Y%m%d_%H%M%S")
    report_dir.mkdir(parents=True, exist_ok=True)

    print(
        f"[soak_test] starting: duration={args.duration_sec:.0f}s "
        f"sample_interval={args.sample_interval_sec:.0f}s output={report_dir}")

    # rclpyの使用はrun_nav2_patrolスレッド(BasicNavigator)1箇所に限定する。
    # 当初はsafety_state監視にもrclpy Subscriptionを使っていたが、別スレッドで
    # 別コンテキストを同時にspinさせるとセグメンテーション違反が実際に発生したため
    # (SafetyStateWatcherのdocstring参照)、こちらはrclpyを使わないサブプロセス
    # ポーリング方式にしている。
    rclpy.init()
    safety_watcher = SafetyStateWatcher()

    nav2_tracker = Nav2ResponseTimeTracker(window_sec=args.sample_interval_sec)
    patrol_stop = threading.Event()
    start_time = time.monotonic()
    patrol_thread = threading.Thread(
        target=run_nav2_patrol,
        args=(patrol_stop, nav2_tracker, start_time, DEFAULT_WAYPOINTS),
        daemon=True,
    )
    patrol_thread.start()

    process_tracker = NodeProcessTracker(MONITORED_PROCESSES)

    try:
        next_sample_at = start_time
        while True:
            elapsed = time.monotonic() - start_time
            if elapsed >= args.duration_sec:
                break
            if time.monotonic() >= next_sample_at:
                process_tracker.sample(elapsed)
                safety_watcher.poll(elapsed)
                print(f"[soak_test] sampled at elapsed={elapsed:.0f}s")
                next_sample_at += args.sample_interval_sec
            time.sleep(min(1.0, args.sample_interval_sec))
    except KeyboardInterrupt:
        print("[soak_test] interrupted by user, finalizing report early")
    finally:
        actual_duration = time.monotonic() - start_time
        process_tracker.sample(actual_duration)
        safety_watcher.poll(actual_duration)
        patrol_stop.set()
        patrol_thread.join(timeout=10.0)
        nav2_tracker.finalize(actual_duration)
        rclpy.shutdown()

    report = build_report(
        args,
        process_tracker.summary(),
        safety_watcher.history,
        nav2_tracker.windows,
        actual_duration,
    )
    report_path = report_dir / "report.json"
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(f"[soak_test] report written to {report_path}")


if __name__ == "__main__":
    main()
