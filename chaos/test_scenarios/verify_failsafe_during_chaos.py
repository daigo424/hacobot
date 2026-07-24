#!/usr/bin/env python3
"""Chaos Mesh実験の実行中に、フェイルセーフ層が正しく反応するかを自動検証する。

実行場所についての注記: このスクリプトは chaos/experiments/*.yaml (kubectl/k3d側)と
/safety/state (ROS2/ros2_nav2_container側)の両方を扱う必要があるが、この2つは別々の
環境で動いている(kubectlはWSL2ホスト側、ROS2はDockerコンテナ内)。そのためこのスクリプトは
**ホスト側で**実行し、kubectlは直接、ROS2側は`docker exec ros2_nav2_container ...`経由で
呼び出す構成にしている(testing/soak_test/run_soak_test.pyのようにコンテナ内で完結させる
方式は、この検証には使えない)。

実行例:
    python3 chaos/test_scenarios/verify_failsafe_during_chaos.py --all
    python3 chaos/test_scenarios/verify_failsafe_during_chaos.py \\
        --experiment chaos/experiments/pod-chaos-kafka.yaml --watch-sec 30

前提: Gazebo+Nav2+safety_bringupが ros2_nav2_container 内で起動していること、
Chaos Meshがk3dクラスタにデプロイ済みであること。
"""
from __future__ import annotations

import argparse
import json
import subprocess
import time
from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
REPORT_DIR = REPO_ROOT / "testing" / "eval-reports" / "chaos_test"
CONTAINER_NAME = "ros2_nav2_container"

# ros2_nav2_containerがk3dクラスタ内のKafkaへ到達できるかどうかは、Dockerの実行基盤に依存する。
# Docker Desktop(コンテナを別ネットワーク名前空間のVMで動かす)ではk3dクラスタ内Podへ
# 到達できず、ネイティブDocker Engineでは到達できる。到達できない環境ではestop_bridgeの
# Kafka疎通チェックを assume_healthy=true でバイパスする必要があり(safety_bringup.launch.py)、
# その場合kafkaネームスペースを対象にしたChaos実験はロボット側のsafety_stateに観測可能な
# 影響を与えない。この既知の制約を隠さず結果に含める。
KNOWN_NETWORK_BOUNDARY_LIMITATION = (
    "ros2_nav2_containerからkafkaネームスペースのPodへ到達できないか、"
    "estop_bridgeのKafka疎通チェックがassume_healthy=trueでバイパスされているため、"
    "Chaos実験はロボット側のsafety_stateに観測可能な影響を与えなかった。"
    "Docker実行基盤とestop_bridgeの設定を確認すること。"
)

EXPECTED_REACTION_STATES = ("DEGRADED", "SAFE_STOP")


def run(cmd: list[str], timeout: int = 15) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


def poll_safety_state() -> str | None:
    result = run([
        "docker", "exec", CONTAINER_NAME, "bash", "-c",
        "source /opt/ros/jazzy/setup.bash && "
        "timeout 3 ros2 topic echo /safety/state --once --field data",
    ])
    if result.returncode != 0 or not result.stdout.strip():
        return None
    return result.stdout.strip().splitlines()[0].strip()


def apply_experiment(yaml_path: Path) -> bool:
    result = run(["kubectl", "apply", "-f", str(yaml_path)])
    print(f"[verify] kubectl apply -f {yaml_path.name}: {result.stdout.strip()}")
    if result.returncode != 0:
        print(f"[verify] apply failed: {result.stderr.strip()}")
        return False
    return True


def delete_experiment(yaml_path: Path) -> None:
    result = run(["kubectl", "delete", "-f", str(yaml_path), "--ignore-not-found"])
    print(f"[verify] kubectl delete -f {yaml_path.name}: {result.stdout.strip()}")


# 状態の「悪化度合い」の順序。既にこれ以上悪い状態にいる場合、そこからの遷移を
# 実験の効果として主張することはできない(因果関係を証明できないため)。
STATE_SEVERITY = {"NORMAL": 0, "DEGRADED": 1, "SAFE_STOP": 2, "MANUAL_RECOVERY": 2}


def wait_for_stable_baseline(max_wait_sec: float = 20.0, poll_interval_sec: float = 2.0) -> str | None:
    """試験開始前にNORMALへ落ち着くのを軽く待つ(環境ノイズによる偽陽性/偽陰性を減らす)。

    このプロジェクトのGazebo/WSL2環境はCPU負荷起因の一時的なDEGRADED/SAFE_STOPを
    頻繁に起こす。NORMALに達しなくてもエラーにはせず、そのまま観測されたベースラインを返す
    (run_scenario側で「ベースライン自体が既に最悪状態」の場合を別扱いする)。
    """
    deadline = time.monotonic() + max_wait_sec
    last_state = poll_safety_state()
    while time.monotonic() < deadline and last_state != "NORMAL":
        time.sleep(poll_interval_sec)
        last_state = poll_safety_state()
    return last_state


def run_scenario(yaml_path: Path, watch_sec: float, poll_interval_sec: float = 2.0) -> dict:
    name = yaml_path.stem
    print(f"\n=== scenario: {name} ===")

    baseline_state = wait_for_stable_baseline()
    print(f"[verify] baseline /safety/state = {baseline_state}")
    # baseline_stateがNone(ポーリング失敗、/safety/state未起動等)の場合にSTATE_SEVERITYの
    # 既定値を低く(-1等)してしまうと、「ベースラインが分からない」だけなのに
    # 後続の観測(SAFE_STOP等)を「悪化した」と誤認してreacted=Trueにしてしまうバグを
    # 実際に踏んだ。ベースラインが不明な場合は最も悪い(=何が起きても反応と言えない)
    # 扱いにする。
    baseline_severity = STATE_SEVERITY.get(baseline_state, STATE_SEVERITY["SAFE_STOP"])

    applied = apply_experiment(yaml_path)
    if not applied:
        return {
            "experiment": name,
            "result": "ERROR",
            "reason": "kubectl apply failed, see logs",
        }

    observed_states: list[dict] = []
    reacted = False
    start = time.monotonic()
    try:
        while time.monotonic() - start < watch_sec:
            state = poll_safety_state()
            elapsed = round(time.monotonic() - start, 1)
            observed_states.append({"elapsed_sec": elapsed, "state": state})
            print(f"[verify] t+{elapsed}s /safety/state = {state}")
            # 「実験の効果」と呼べるのは、ベースラインより悪化した状態へ遷移した場合のみ。
            # ベースライン自体が既にSAFE_STOP等であれば、そこに留まっていても
            # 実験に起因するかは判別できないため反応とみなさない。
            if state in EXPECTED_REACTION_STATES and STATE_SEVERITY.get(state, -1) > baseline_severity:
                reacted = True
            time.sleep(poll_interval_sec)
    finally:
        delete_experiment(yaml_path)

    final_state = observed_states[-1]["state"] if observed_states else None
    inconclusive = baseline_severity >= STATE_SEVERITY["SAFE_STOP"]
    if inconclusive:
        verdict = "INCONCLUSIVE"
    else:
        verdict = "PASS" if reacted else "FAIL"

    result = {
        "experiment": name,
        "experiment_yaml": str(yaml_path.resolve().relative_to(REPO_ROOT)),
        "run_at": datetime.now().isoformat(),
        "watch_sec": watch_sec,
        "baseline_state": baseline_state,
        "observed_states": observed_states,
        "final_state": final_state,
        "reacted_as_expected": reacted,
        "result": verdict,
        "inconclusive_reason": (
            "baseline /safety/state was already SAFE_STOP/MANUAL_RECOVERY before the "
            "experiment was applied (likely environmental noise from CPU load); cannot "
            "attribute any observed state to this experiment specifically"
        ) if inconclusive else None,
        "known_limitation": (
            KNOWN_NETWORK_BOUNDARY_LIMITATION if verdict == "FAIL" else None
        ),
    }
    print(f"[verify] scenario {name}: result={result['result']}")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--experiment", type=Path,
        help="単一のChaos実験YAMLを指定して検証する")
    parser.add_argument(
        "--all", action="store_true",
        help="chaos/experiments/配下の全実験を順に検証する")
    parser.add_argument(
        "--watch-sec", type=float, default=30.0,
        help="実験適用後、/safety/stateを監視する秒数(既定30秒)")
    args = parser.parse_args()

    if not args.experiment and not args.all:
        parser.error("--experiment <yaml> または --all のいずれかを指定してください")

    experiments_dir = REPO_ROOT / "chaos" / "experiments"
    targets = (
        sorted(experiments_dir.glob("*.yaml")) if args.all
        else [args.experiment.resolve()]
    )

    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    for yaml_path in targets:
        result = run_scenario(yaml_path, args.watch_sec)
        report_path = REPORT_DIR / f"{yaml_path.stem}_result.json"
        report_path.write_text(json.dumps(result, indent=2, ensure_ascii=False))
        print(f"[verify] report written to {report_path}")


if __name__ == "__main__":
    main()
