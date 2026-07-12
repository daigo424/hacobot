#!/usr/bin/env bash
set -e

CLUSTER_NAME="${CLUSTER_NAME:-hacobot-cluster}"

if ! k3d cluster list "$CLUSTER_NAME" >/dev/null 2>&1; then
    echo "k3d cluster '$CLUSTER_NAME' は存在しません。削除をスキップします。"
    exit 0
fi

echo "k3dクラスタ '$CLUSTER_NAME' を削除します(Kafka/SeaweedFS/Chaos Meshのデータも含めて全て削除されます)..."
k3d cluster delete "$CLUSTER_NAME"

echo "削除が完了しました。"
