#!/usr/bin/env bash
# demos/01〜06を順番に実行する。1つでも失敗したら止まる。
#
# 使い方:
#   bash demos/run_all.sh          # 全部
#   bash demos/run_all.sh 1 2 3    # 手順1〜3だけ
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

ALL_STEPS=(01_build_test 02_launch_stack 03_break_and_verify_failsafe 04_observability 05_chaos_experiment 06_soak_test 07_real_kafka_estop)

if [ "$#" -gt 0 ]; then
  steps=()
  for n in "$@"; do
    idx=$((n - 1))
    steps+=("${ALL_STEPS[$idx]}")
  done
else
  steps=("${ALL_STEPS[@]}")
fi

for step in "${steps[@]}"; do
  echo
  echo "=================================================="
  echo " $step"
  echo "=================================================="
  if ! bash "./${step}.sh"; then
    echo
    echo "!! ${step} が失敗しました。ここで停止します。"
    exit 1
  fi
done

echo
echo "=================================================="
echo " 全ての手順が成功しました"
echo "=================================================="
