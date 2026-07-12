#!/usr/bin/env bash
set -e

CLUSTER_NAME="${CLUSTER_NAME:-hacobot-cluster}"

if k3d cluster list "$CLUSTER_NAME" >/dev/null 2>&1; then
    echo "k3d cluster '$CLUSTER_NAME' はすでに存在します。作成をスキップします。"
    exit 0
fi

echo "k3dクラスタ '$CLUSTER_NAME' をWSL2上に構築します..."

# --port 30092/30093: Kafkaのnodeportリスナー(infra/k8s/kafka/kafka-cluster.yaml)を
# ホスト側に公開する。ros2_nav2_container(edge/docker/、network_mode: host)から
# 127.0.0.1:30092経由でKafkaへ到達できるようにするため
# (ネイティブDocker Engine使用時のみ有効。Docker Desktop使用時は別ネットワーク
# 名前空間のため到達不可)。
k3d cluster create "$CLUSTER_NAME" --gpus=all \
    --volume /usr/lib/wsl/lib:/usr/lib/wsl/lib@server:0 \
    --port 30092:30092@server:0 \
    --port 30093:30093@server:0

echo "クラスタ '$CLUSTER_NAME' の準備が完了しました。"
kubectl cluster-info
