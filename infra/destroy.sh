#!/usr/bin/env bash
set -e

# infra/deploy.sh の逆順で後片付けする(Kafka→Helmリリースの順)。
# Strimzi OperatorがKafka CRDの削除処理(Pod/PVCのクリーンアップ)を担うため、
# 先にOperatorを消すとKafkaクラスタが正しく片付かない。deploy.shが
# 「Operatorを入れてからKafkaを作る」順序である以上、削除は逆に
# 「Kafkaを消してからOperatorを消す」順序にする必要がある。
#
# k3dクラスタ自体はここでは削除しない(Kafka/SeaweedFS/Chaos Meshのリリースだけを
# 片付けて、クラスタは再利用したい場合があるため)。クラスタごと削除する場合は
# `bash infra/k3d/teardown.sh` を直接使うこと。

INFRA_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# helmfileはsudo権限なしで ~/.local/bin に導入しているため、
# シェルの.bashrc反映(新規ターミナル起動 or source)に依存せずここでPATHを通す
export PATH="$HOME/.local/bin:$PATH"

if ! command -v helmfile >/dev/null 2>&1; then
    echo "helmfileが見つかりません。~/.local/bin/helmfile が存在するか確認してください。" >&2
    exit 1
fi

if ! kubectl cluster-info >/dev/null 2>&1; then
    echo "k3dクラスタに接続できません(未構築、または既に削除済み)。後片付けをスキップします。"
    exit 0
fi

echo "[1/2] Kafkaクラスタ・トピックを削除します..."
kubectl delete -f "$INFRA_ROOT/k8s/kafka/kafka-topics.yaml" --ignore-not-found
kubectl delete -f "$INFRA_ROOT/k8s/kafka/kafka-cluster.yaml" --ignore-not-found

echo "[2/2] Helmリリース(SeaweedFS, Strimzi Operator, Chaos Mesh)を削除します..."
helmfile -f "$INFRA_ROOT/helmfile.yaml" destroy

echo "完了しました(k3dクラスタ自体は残しています。クラスタごと削除する場合は 'bash infra/k3d/teardown.sh' を使ってください)。"
kubectl get pods -A
