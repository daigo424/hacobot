#!/usr/bin/env bash
# docs/verification_guide.md 手順6: 耐久試験スクリプト(短時間ドライラン)
#
# 成功の目安: testing/soak_test/reports/[日時]/report.json が生成され、
# process_memory等のフィールドが埋まっている。
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
source lib/common.sh

require_container

DURATION="${DURATION:-60s}"
SAMPLE_INTERVAL="${SAMPLE_INTERVAL:-15s}"

log "6. 耐久試験スクリプトをドライラン実行します(duration=$DURATION)..."

before="$(docker exec "$CONTAINER_NAME" bash -c \
  "ls /testing/soak_test/reports 2>/dev/null" || true)"

if ! docker exec "$CONTAINER_NAME" bash -c \
    "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && \
     cd /testing/soak_test && \
     python3 run_soak_test.py --duration $DURATION --sample-interval $SAMPLE_INTERVAL"; then
  fail "run_soak_test.pyが異常終了しました"
  exit 1
fi

latest_dir="$(docker exec "$CONTAINER_NAME" bash -c \
  "ls -t /testing/soak_test/reports | head -1")"
report_path="/testing/soak_test/reports/${latest_dir}/report.json"

log "生成されたレポート: $report_path"

check_result="$(docker exec "$CONTAINER_NAME" python3 -c "
import json
with open('$report_path') as f:
    data = json.load(f)
required = ['process_memory', 'safety_state_history', 'nav2_response_time_windows', 'pass_criteria']
missing = [k for k in required if k not in data]
if missing:
    print('MISSING:' + ','.join(missing))
else:
    print('OK')
" 2>&1)"

if [ "$check_result" = "OK" ]; then
  pass "report.jsonに期待するフィールドが全て含まれています"
  echo "  ホスト側から見る場合: cat $REPO_ROOT${report_path} | python3 -m json.tool"
else
  fail "report.jsonの検証に失敗しました: $check_result"
  exit 1
fi
