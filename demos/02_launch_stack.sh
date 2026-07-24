#!/usr/bin/env bash
# docs/verification_guide.md 手順2: Gazebo + Nav2 + フェイルセーフ層の起動確認
#
# 成功の目安: heartbeat_monitor / safety_state_machine / watchdog /
# nav2_heartbeat_adapter / estop_bridge / bt_navigator などが ros2 node list に出る
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
source lib/common.sh

require_container

EXPECTED_NODES=(
  heartbeat_monitor
  safety_state_machine
  watchdog
  nav2_heartbeat_adapter
  estop_bridge
  bt_navigator
)

node_list() {
  ros2_exec "ros2 node list" 2>/dev/null
}

current_nodes="$(node_list || true)"
missing=()
for n in "${EXPECTED_NODES[@]}"; do
  echo "$current_nodes" | grep -q "/$n" || missing+=("$n")
done
# 同名ノードの重複(=nav2_bringup.launch.pyが多重起動している状態)が無いかも確認する。
# node_list()だけでは「起動済みかどうか」の判定が甘く、実際に多重起動を
#見逃して二重にlaunchしてしまったことがあるため、重複が1件でもあれば
# 「クリーンではない」とみなして作り直す。
has_duplicates=false
if [ -n "$current_nodes" ] && [ "$(echo "$current_nodes" | sort | uniq -d | wc -l)" -gt 0 ]; then
  has_duplicates=true
fi

if [ ${#missing[@]} -eq 0 ] && ! $has_duplicates; then
  pass "既に全ノードが起動済みです(再起動はスキップします)"
  echo "$current_nodes"
  exit 0
fi

if $has_duplicates || [ -n "$current_nodes" ]; then
  warn "中途半端な状態(ノード重複、または一部だけ起動済み)を検知しました。"
  warn "docker compose restart でコンテナごとクリーンな状態に戻します"
  ( cd "$REPO_ROOT/edge/docker" && docker compose restart ros2-nav2 )
  sleep 5
fi

log "2. Gazeboを起動します..."
docker exec -d "$CONTAINER_NAME" bash -c \
  "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \
   ros2 launch nav2_bringup_custom gazebo_sim.launch.py > /tmp/demo_gazebo.log 2>&1"
sleep 15

# GUIは重いだけで仕組みに影響しないので落とす。gazebo_sim.launch.py(turtlebot3本家)は
# server(-s)/GUI(-g)を別プロセスとして起動するため"gz sim -g"だが、create_world.launch.py
# (spawn_robotの新しいマルチロボット経路)は分割せず"gz sim server"/"gz sim gui"という
# 名前の子プロセスに分かれるため、両方にマッチするパターンにする(片方だけだとGUIが
# 残ったままになるか、経路によってはサーバーごと巻き添えで落ちる)
docker exec "$CONTAINER_NAME" bash -c \
  "ps aux | grep -E 'gz sim (gui|-g( |\$))' | grep -v grep | awk '{print \$2}' | xargs -r kill -9" >/dev/null 2>&1

log "Nav2 + safety_bringup(5ノード)を起動します..."
docker exec -d "$CONTAINER_NAME" bash -c \
  "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \
   ros2 launch nav2_bringup_custom nav2_bringup.launch.py > /tmp/demo_nav2.log 2>&1"

log "起動を待っています(最大60秒)..."
for i in $(seq 1 12); do
  sleep 5
  current_nodes="$(node_list || true)"
  missing=()
  for n in "${EXPECTED_NODES[@]}"; do
    echo "$current_nodes" | grep -q "/$n" || missing+=("$n")
  done
  if [ ${#missing[@]} -eq 0 ]; then
    break
  fi
done

if [ ${#missing[@]} -eq 0 ]; then
  pass "期待するノードが全て起動しました:"
  for n in "${EXPECTED_NODES[@]}"; do echo "  - /$n"; done
else
  fail "以下のノードが見つかりません: ${missing[*]}"
  echo "ログを確認してください: docker exec $CONTAINER_NAME cat /tmp/demo_gazebo.log /tmp/demo_nav2.log"
  exit 1
fi
