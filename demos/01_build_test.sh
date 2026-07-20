#!/usr/bin/env bash
# docs/verification_guide.md 手順1: ビルド・単体テスト
#
# 成功の目安: `Summary: 9 packages finished` と
# `Summary: 21 tests, 0 errors, 0 failures, 0 skipped`
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
source lib/common.sh

log "1. ワークスペース全体をビルド・テストします"

cd "$REPO_ROOT"
make up

# ライブ稼働中の安全ノードが残っていると、gtestと同じDDSトピック
# (/safety/anomaly_event等)で混信して見せかけのテスト失敗になることがある。
# colcon test前に必ず一掃しておく。
if docker ps --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
  docker exec "$CONTAINER_NAME" bash -c \
    "pkill -9 -f 'heartbeat_monitor_node|safety_state_machine_node|watchdog_node|estop_bridge_node|nav2_heartbeat_adapter_node' 2>/dev/null || true"
fi

if ! make build-test; then
  fail "ビルドまたはテストが失敗しました(上記のcolconログを確認してください)"
  exit 1
fi

pass "全パッケージのビルド・テストが成功しました"
