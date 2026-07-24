#!/usr/bin/env bash
# docs/verification_guide.md 手順7: Kafka経由の本物のリモートE-Stopをエンドツーエンドで確認する
#
# ネイティブDocker Engine(Docker Desktopではない)+ k3dのnodeportリスナー
# (infra/k8s/kafka/kafka-cluster.yaml)により、ros2_nav2_containerから実際に
# k3dクラスタ内のKafkaへ到達できる。これによりリモートE-Stopの実配信経路を、
# 本物のKafkaメッセージでエンドツーエンドに検証できる。
#
# 成功の目安: kafka-console-producer.shで"ESTOP"を送信すると、
# estop_bridgeの実際のKafkaコンシューマスレッドがそれを受信し、
# /safety/state が SAFE_STOP へ遷移する。
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
source lib/common.sh

require_container

if ! command -v kubectl >/dev/null 2>&1; then
  fail "kubectlが見つかりません"
  exit 1
fi

KAFKA_POD="$(kubectl get pods -n kafka -l strimzi.io/name=hacobot-kafka-kafka \
  -o jsonpath='{.items[0].metadata.name}' 2>/dev/null)"
if [ -z "$KAFKA_POD" ]; then
  fail "Kafka Podが見つかりません。先に bash infra/deploy.sh を実行してください。"
  exit 1
fi

if ! docker exec "$CONTAINER_NAME" bash -c \
    "source /opt/ros/jazzy/setup.bash && ros2 node list 2>/dev/null | grep -q /estop_bridge"; then
  fail "estop_bridgeが起動していません。先に demos/02_launch_stack.sh を実行してください。"
  exit 1
fi

log "7. estop_bridgeのKafka疎通状態を確認します..."
reachable="$(docker exec "$CONTAINER_NAME" bash -c \
  "timeout 3 bash -c '</dev/tcp/127.0.0.1/30092' 2>&1 && echo REACHABLE || echo UNREACHABLE")"
if [ "$reachable" != "REACHABLE" ]; then
  fail "127.0.0.1:30092(KafkaのNodePort)へ到達できません。"
  echo "  Docker Desktopを使っている場合はこの検証は成立しません(ネイティブDocker Engineが必要)。"
  echo "  k3dクラスタがnodeportリスナー込みで構築されているか確認してください: bash infra/deploy.sh"
  exit 1
fi
pass "127.0.0.1:30092 に到達できています"

echo "現在の /safety/state:"
docker exec "$CONTAINER_NAME" bash -c \
  "source /opt/ros/jazzy/setup.bash && timeout 3 ros2 topic echo /safety/state --once --field data" 2>/dev/null \
  | head -1

# この検証環境はgzserverの高CPU負荷でセンサー/ハートビートが本物のノイズで
# 途絶することがあり、既にSAFE_STOPの場合は単純な状態比較では
# 「Kafka経由のE-Stopが効いたこと」を証明できない(chaos/test_scenarios/
# verify_failsafe_during_chaos.pyで踏んだのと同じ因果関係の問題)。
# そのため、ログに実際に"estop_bridge"が原因のCRITICALが記録されたかで判定する。
LOG_PATH="/tmp/demo_nav2.log"
lines_before="$(docker exec "$CONTAINER_NAME" bash -c "wc -l < $LOG_PATH" 2>/dev/null || echo 0)"

log "Kafkaの robot-control トピックへ本物の \"ESTOP\" メッセージを送信します..."
if ! echo "ESTOP" | kubectl exec -i -n kafka "$KAFKA_POD" -c kafka -- \
    bin/kafka-console-producer.sh --bootstrap-server localhost:9092 --topic robot-control 2>&1; then
  fail "Kafkaへのメッセージ送信に失敗しました"
  exit 1
fi

log "estop_bridgeがKafka経由で受信し、CRITICAL通知を出すのを待っています..."
attributed=false
for i in $(seq 1 10); do
  sleep 1
  new_lines="$(docker exec "$CONTAINER_NAME" bash -c \
    "tail -n +$((lines_before + 1)) $LOG_PATH 2>/dev/null" || true)"
  if echo "$new_lines" | grep -q "CRITICAL anomaly from 'estop_bridge'"; then
    attributed=true
    break
  fi
done

final_state="$(docker exec "$CONTAINER_NAME" bash -c \
  "source /opt/ros/jazzy/setup.bash && timeout 3 ros2 topic echo /safety/state --once --field data" 2>/dev/null \
  | head -1 | tr -d '\r')"
echo "最終的な /safety/state: ${final_state:-<取得失敗>}"

if $attributed; then
  pass "本物のKafkaメッセージ経由のE-Stopが実際にestop_bridge発のCRITICALとして処理されたことをログで確認しました"
else
  fail "estop_bridge起因のCRITICALをログで確認できませんでした(Kafkaメッセージが届いていない可能性があります)"
  exit 1
fi

echo
log "復旧する場合(2回送るとNORMALへ戻ります):"
echo "  docker exec $CONTAINER_NAME bash -c \\"
echo "    \"source /opt/ros/jazzy/setup.bash && \\"
echo "     ros2 topic pub -1 /safety/recovery_command std_msgs/msg/Empty '{}'\""
