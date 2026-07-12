#!/usr/bin/env bash
# docs/verification_guide.md 手順4: 可観測性(Prometheus/Grafana)
#
# 成功の目安: Prometheusが5ノード全てをup状態でスクレイプしており、
# Grafanaにhacobot_health.jsonダッシュボードが自動プロビジョニングされている。
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
source lib/common.sh

require_container

log "4. Prometheus/Grafanaを起動します..."
(cd "$REPO_ROOT/edge/docker" && docker compose up -d prometheus grafana)

log "起動を待っています..."
sleep 10

# Docker Desktop使用時はネットワークVM境界により、ホストシェルからではなく
# ros2_nav2_container経由でcurlする必要がある(ネイティブDocker Engineならホストから直接でも可)
log "Prometheusのスクレイプ対象を確認します..."
targets_json="$(docker exec "$CONTAINER_NAME" curl -s --max-time 5 http://localhost:9090/api/v1/targets || true)"

if [ -z "$targets_json" ]; then
  fail "Prometheus(http://localhost:9090)に接続できませんでした"
  exit 1
fi

down_targets="$(echo "$targets_json" | python3 -c "
import json, sys
data = json.load(sys.stdin)
targets = data.get('data', {}).get('activeTargets', [])
for t in targets:
    print(f\"{t['scrapeUrl']} {t['health']}\")
" 2>/dev/null)"

echo "$down_targets"
up_count="$(echo "$down_targets" | grep -c ' up$' || true)"

if [ "$up_count" -ge 5 ]; then
  pass "Prometheusが5ノード全てをスクレイプできています(up: $up_count)"
else
  fail "upなターゲットが $up_count 件しかありません(5件必要)。safety_bringupが起動しているか確認してください"
  exit 1
fi

log "Grafanaのダッシュボード自動プロビジョニングを確認します..."
search_json="$(docker exec "$CONTAINER_NAME" curl -s --max-time 5 -u admin:admin \
  'http://localhost:3001/api/search?query=hacobot' || true)"

if echo "$search_json" | grep -q 'hacobot-health'; then
  pass "Grafanaに hacobot System Health ダッシュボードが登録されています"
  echo "  ブラウザで http://localhost:3001 (admin/admin) を開いて確認できます"
else
  fail "Grafana上でダッシュボードが見つかりませんでした"
  echo "$search_json"
  exit 1
fi
