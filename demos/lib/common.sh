#!/usr/bin/env bash
# demos/*.sh 共通ヘルパー。各デモスクリプトの先頭で `source`して使う。
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CONTAINER_NAME="${CONTAINER_NAME:-ros2_nav2_container}"

COLOR_GREEN='\033[0;32m'
COLOR_RED='\033[0;31m'
COLOR_YELLOW='\033[0;33m'
COLOR_RESET='\033[0m'

log()  { echo -e "[demo] $*"; }
pass() { echo -e "${COLOR_GREEN}[PASS]${COLOR_RESET} $*"; }
fail() { echo -e "${COLOR_RED}[FAIL]${COLOR_RESET} $*"; }
warn() { echo -e "${COLOR_YELLOW}[WARN]${COLOR_RESET} $*"; }

# ros2_nav2_container内でコマンドを実行する(ROS2環境をsourceした状態で)
ros2_exec() {
  docker exec "$CONTAINER_NAME" bash -c \
    "source /opt/ros/jazzy/setup.bash && \
     ( [ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash || true ) && \
     $1"
}

# ros2_nav2_containerがそもそも動いているか確認する。動いていなければ案内して終了する。
require_container() {
  if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
    fail "コンテナ '$CONTAINER_NAME' が起動していません。"
    echo "  次を実行してから再度お試しください: make up"
    exit 1
  fi
}
