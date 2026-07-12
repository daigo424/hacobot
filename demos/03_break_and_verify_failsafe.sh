#!/usr/bin/env bash
# docs/verification_guide.md 手順3: フェイルセーフの核心 -- 実際に壊してSAFE_STOPになるか
#
# Nav2プロセスを意図的にkillし、/safety/state が
# NORMAL -> DEGRADED -> SAFE_STOP と変化することを確認する。
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
source lib/common.sh

require_container

WATCH_SEC="${WATCH_SEC:-20}"

safety_state() {
  timeout 3 docker exec "$CONTAINER_NAME" bash -c \
    "source /opt/ros/humble/setup.bash && \
     timeout 2 ros2 topic echo /safety/state --once --field data" 2>/dev/null \
    | head -1 | tr -d '\r'
}

if ! docker exec "$CONTAINER_NAME" bash -c \
    "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null | grep -q /safety_state_machine"; then
  fail "safety_state_machineが起動していません。先に demos/02_launch_stack.sh を実行してください。"
  exit 1
fi

baseline="$(safety_state)"
log "現在の /safety/state = ${baseline:-<取得失敗>}"

log "Nav2本体(component_container_isolated)を意図的にkillします..."
docker exec "$CONTAINER_NAME" bash -c "pkill -9 -f component_container_isolated" || true

log "${WATCH_SEC}秒間、/safety/state を監視します..."
saw_degraded=false
saw_safe_stop=false
for i in $(seq 1 "$WATCH_SEC"); do
  sleep 1
  state="$(safety_state)"
  echo "  t+${i}s: ${state:-<取得失敗>}"
  [ "$state" = "DEGRADED" ] && saw_degraded=true
  if [ "$state" = "SAFE_STOP" ]; then
    saw_safe_stop=true
    break
  fi
done

if $saw_safe_stop; then
  pass "SAFE_STOPへの遷移を確認しました(DEGRADED経由: $saw_degraded)"
else
  fail "監視時間内にSAFE_STOPへ遷移しませんでした(WATCH_SEC=$WATCH_SEC を増やして再試行してみてください)"
  exit 1
fi

echo
log "復旧を試す場合は以下を実行してください(2回送るとNORMALへ戻ります。"
log "ただしNav2は死んだままなので、根本原因が直っていなければ再びSAFE_STOPへ戻るのが正しい挙動です):"
echo "  docker exec $CONTAINER_NAME bash -c \\"
echo "    \"source /opt/ros/humble/setup.bash && \\"
echo "     ros2 topic pub -1 /safety/recovery_command std_msgs/msg/Empty '{}'\""
