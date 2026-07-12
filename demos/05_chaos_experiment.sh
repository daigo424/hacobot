#!/usr/bin/env bash
# docs/verification_guide.md 手順5: カオス実験(k3d側)
#
# 成功の目安1: pod-chaos-kafka.yaml を適用すると、KafkaのBroker PodのAGEが
# 一瞬0にリセットされ(強制終了され)、その後自動的に再起動して1/1 Runningに戻る。
# 成功の目安2: ネイティブDocker Engine + k3dのnodeportリスナー
# (infra/k8s/kafka/kafka-cluster.yaml)により、ros2_nav2_containerから実際に
# Kafkaへ到達できる環境では、Kafka Podが落ちている間にestop_bridgeの
# TCP到達性チェックが reachable -> unreachable を検知するログが出ることも確認する
# (これがロボット側フェイルセーフへの実際の波及の入り口になる)。
#
# 注意: Docker Desktopを使っている場合、成功の目安2は観測できない
# (ネットワーク名前空間が分離されているため)。その場合でも
# 目安1(Kafka自体の自己修復)は確認できる。
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
source lib/common.sh

if ! command -v kubectl >/dev/null 2>&1; then
  fail "kubectlが見つかりません"
  exit 1
fi

if ! kubectl get ns kafka >/dev/null 2>&1; then
  fail "kafka namespaceが見つかりません。先に bash infra/deploy.sh を実行してください。"
  exit 1
fi

if ! kubectl get ns chaos-mesh >/dev/null 2>&1; then
  fail "chaos-mesh namespaceが見つかりません。先に以下を実行してください:"
  echo "  export PATH=\"\$HOME/.local/bin:\$PATH\""
  echo "  helmfile -f infra/helmfile.yaml sync"
  exit 1
fi

pod_before="$(kubectl get pods -n kafka -l strimzi.io/name=hacobot-kafka-kafka \
  -o jsonpath='{.items[0].metadata.name}')"
age_before="$(kubectl get pod "$pod_before" -n kafka -o jsonpath='{.status.startTime}')"
log "実験前のKafka Broker Pod: $pod_before (起動時刻: $age_before)"

# ロボット側への波及を観測できるかどうかは、estop_bridgeが起動しておりKafkaへ
# 到達できているかどうかで決まる(demos/07_real_kafka_estop.shと同じ判定)。
robot_side_observable=false
if docker ps --format '{{.Names}}' | grep -qx "$CONTAINER_NAME" && \
   docker exec "$CONTAINER_NAME" bash -c \
     "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null | grep -q /estop_bridge" && \
   [ "$(docker exec "$CONTAINER_NAME" bash -c \
     "timeout 3 bash -c '</dev/tcp/127.0.0.1/30092' 2>&1 && echo R || echo U")" = "R" ]; then
  robot_side_observable=true
fi

LOG_PATH="/tmp/demo_nav2.log"
lines_before=0
if $robot_side_observable; then
  lines_before="$(docker exec "$CONTAINER_NAME" bash -c "wc -l < $LOG_PATH" 2>/dev/null || echo 0)"
fi

log "5. pod-chaos-kafka.yaml を適用します..."
kubectl apply -f "$REPO_ROOT/chaos/experiments/pod-chaos-kafka.yaml"

log "Podが再起動されるのを待っています(最大30秒)..."
restarted=false
connectivity_lost_detected=false
for i in $(seq 1 15); do
  sleep 2
  age_now="$(kubectl get pod "$pod_before" -n kafka -o jsonpath='{.status.startTime}' 2>/dev/null || true)"
  if [ -n "$age_now" ] && [ "$age_now" != "$age_before" ]; then
    restarted=true
  fi
  # Pod名自体が変わる(StrimziPodSetが新しいPodを作る)場合もある
  if ! kubectl get pod "$pod_before" -n kafka >/dev/null 2>&1; then
    restarted=true
  fi

  if $robot_side_observable && ! $connectivity_lost_detected; then
    new_lines="$(docker exec "$CONTAINER_NAME" bash -c \
      "tail -n +$((lines_before + 1)) $LOG_PATH 2>/dev/null" || true)"
    if echo "$new_lines" | grep -q "Kafka connectivity.*reachable -> unreachable"; then
      connectivity_lost_detected=true
      log "estop_bridgeがKafka接続断を検知したログを確認しました"
    fi
  fi

  if $restarted && ( ! $robot_side_observable || $connectivity_lost_detected ); then
    break
  fi
done

kubectl delete -f "$REPO_ROOT/chaos/experiments/pod-chaos-kafka.yaml" --ignore-not-found

log "Kafka Podが1/1 Runningへ戻るのを待っています..."
kubectl wait --for=condition=ready pod -l strimzi.io/name=hacobot-kafka-kafka -n kafka --timeout=60s || true
kubectl get pods -n kafka

if $restarted; then
  pass "PodChaosによりKafka Broker Podが実際にkillされ、再起動されたことを確認しました"
else
  fail "Podの再起動を検知できませんでした(タイミングの問題の可能性があります。kubectl get pods -n kafka で手動確認してください)"
  exit 1
fi

if $robot_side_observable; then
  if $connectivity_lost_detected; then
    pass "ロボット側(estop_bridge)がKafka接続断を実際に検知したことをログで確認しました"
  else
    warn "ロボット側の接続断ログを検知タイミング内に確認できませんでした(Kafkaの復旧が速すぎた可能性があります)"
  fi
else
  warn "estop_bridgeが起動していないか、Kafkaへの疎通が無いため、ロボット側への波及は確認していません。"
  warn "先に demos/02_launch_stack.sh を実行し、demos/07_real_kafka_estop.sh でKafka疎通を確認してください。"
fi
