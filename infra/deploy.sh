#!/usr/bin/env bash
set -e

INFRA_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# helmfileはsudo権限なしで ~/.local/bin に導入しているため、
# シェルの.bashrc反映(新規ターミナル起動 or source)に依存せずここでPATHを通す
export PATH="$HOME/.local/bin:$PATH"

if ! command -v helmfile >/dev/null 2>&1; then
    echo "helmfileが見つかりません。~/.local/bin/helmfile が存在するか確認してください。" >&2
    exit 1
fi

echo "[1/3] k3dクラスタを準備します..."
bash "$INFRA_ROOT/k3d/setup.sh"

echo "[2/3] Helmリリース(SeaweedFS, Strimzi Operator)を同期します..."
helmfile -f "$INFRA_ROOT/helmfile.yaml" sync

echo "[3/3] Kafkaクラスタ・トピックを適用します..."
kubectl apply -f "$INFRA_ROOT/k8s/kafka/kafka-cluster.yaml"
kubectl apply -f "$INFRA_ROOT/k8s/kafka/kafka-topics.yaml"

echo "Kafkaクラスタの準備を待っています..."
kubectl wait --namespace kafka --for=condition=ready kafka/hacobot-kafka --timeout=300s

echo "完了しました。"
kubectl get pods -n seaweedfs
kubectl get pods -n kafka
kubectl get kafkatopics -n kafka
