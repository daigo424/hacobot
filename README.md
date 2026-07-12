# hacobot

自動運転・配送ロボットシステムの**長時間安定稼働・フェイルセーフ・分散環境での安全な停止制御**を
検証するプロジェクトです。TurtleBot3 + Nav2による自律走行を題材に、通信途絶・センサー故障・
クラウド側障害といった異常系を実際に発生させ、ロボット側が自律的に安全停止へ倒れる仕組みを、
単体テスト・実機統合・カオスエンジニアリングの3段階で検証しています。

## シミュレーション

| Gazebo | RViz |
|---|---|
| ![gazebo](docs/gazebo.png) | ![rviz](docs/rviz.png) |

## 本番想定図

![aws-production-architecture](docs/aws-production-architecture.drawio.png)

## アーキテクチャ図(ローカル動作時)

![architecture-diagram](docs/architecture-diagram.drawio.png)

## 検証結果サマリー

### カオス実験: Chaos Meshによる障害注入

| 実験 | 対象 | 検証したかったこと | 結果 |
|---|---|---|---|
| `pod-chaos-kafka` | Kafka Broker Pod強制終了 | Kafkaがクラッシュしても自律SAFE_STOPへ倒れるか | ✅ Kafka Pod自体は正しくkill→自動復旧することを確認。ロボット側への波及はタイミングが合えば観測可能(下記参照) |
| `network-chaos-robot-cloud` | Kafka Pod経由で遅延250ms+ロス10%を注入 | 通信劣化時にDEGRADEDへ先行遷移するか | ✅ 実験自体は正しく適用・観測できることを確認 |
| `stress-chaos-node` | Kafka PodへCPU/メモリ負荷 | クラウド側高負荷時の挙動 | ✅ 実験自体は正しく適用できることを確認 |

**ロボット側への波及について**: k3dにKafkaのNodePortリスナーを追加
(`infra/k8s/kafka/kafka-cluster.yaml`)することで、`ros2_nav2_container`から実際に
Kafkaへ到達できるようになり、本物のKafkaメッセージで`estop_bridge`経由のE-Stopが
`/safety/state`を`SAFE_STOP`にすることをエンドツーエンドで確認済み
(`demos/07_real_kafka_estop.sh`)。ただしPodChaos実行中の接続断検知はKafkaの復旧速度
(20〜50秒程度)と接続チェック間隔(2秒)のタイミング次第で毎回検知できるとは限らない。
(実行環境による制約は「既知の制約」を参照)

### 実際にロボット側で確認できた壊し方(実機統合テスト)

上記のネットワーク制約が無い部分(ロボット自身のプロセス・センサー)は、実際に壊して
正しく安全停止することを確認済み。

| 壊し方 | 検知経路 | 実際の挙動 |
|---|---|---|
| Nav2プロセスをkill | `nav2_heartbeat_adapter` → `heartbeat_monitor`(WARNING) | NORMAL → DEGRADED(速度制限) → SAFE_STOP(完全停止)と段階的に遷移 |
| カメラ/LiDARトピックの停止 | `watchdog`(CRITICAL) | DEGRADEDを経由せず即座にSAFE_STOP |
| `recovery_command`を根本原因が直っていない状態で送信 | `heartbeat_monitor`/`watchdog`のis_stale_リセット | 偽の`NORMAL`に固定されず、再検知してSAFE_STOPへ戻ることを確認(過去に発見・修正したバグの回帰テストとしても実施) |
| Kafkaの`robot-control`トピックへ本物の`"ESTOP"`メッセージを送信 | `estop_bridge`(Kafkaコンシューマ専用スレッド) | `E-Stop command received` → `CRITICAL anomaly` → 即座にSAFE_STOP(ネイティブDocker Engineでのみ検証可能。`demos/07_real_kafka_estop.sh`) |

### 耐久試験: Soak Test

| 項目 | 状態 |
|---|---|
| `testing/soak_test/run_soak_test.py`の動作確認(60秒ドライラン) | ✅ メモリ計測・再起動検知・安全状態履歴・Nav2応答時間の記録、JSON出力まで一通り完走することを確認 |
| 24時間/72時間の本番耐久試験(メモリ増加率・予期しない再起動回数等の合格基準を設定済み) | 未実施(スクリプトは対応済み。`--duration 24h`/`--duration 72h`で今後実行可能) |

24h/72hの実行には長時間ホストを占有するため、機能検証のみ実施した。

## 技術スタック

| レイヤー | 技術 |
|---|---|
| シミュレーション | Gazebo Classic, TurtleBot3 |
| ナビゲーション | Nav2, slam_toolbox (ROS2 Humble) |
| フェイルセーフ層 | C++ (rclcpp_lifecycle), 独自メッセージ(`safety_msgs`) |
| メッセージング | Kafka (Strimzi Operator) |
| ストレージ | SeaweedFS (S3互換)※ |
| インフラ | k3d, Helm / Helmfile |
| カオスエンジニアリング | Chaos Mesh |
| 可観測性 | Prometheus, Grafana |
| 耐久試験・検証スクリプト | Python (psutil, rclpy, nav2_simple_commander) |

※ SeaweedFSと、Kafkaの`robot-telemetry`トピック(位置情報・画像等のテレメトリ用)は
将来のデータ基盤としてインフラのみ構築済みで、実際にデータを書き込むコードはまだ無い。
安全系(フェイルセーフ層)は`robot-control`(E-Stop)等の別トピックのみを使い、これらには
一切依存しない。

## ローカルセットアップ手順

### 前提

- WSL2 + ネイティブDocker Engine推奨(手順5の一部・手順7に影響。詳細は「既知の制約」参照)
- `kubectl`, `helm`, `helmfile`, `k3d`がインストール済み

### 1. k3dクラスタ + クラウド側基盤の構築

```bash
bash infra/deploy.sh
# SeaweedFS, Strimzi Kafka(robot-telemetry/robot-controlトピック含む)、
# Chaos Meshが全て起動する(infra/helmfile.yamlに一括定義済み)
```

Chaos Meshが起動しているかは以下で確認できる:

```bash
kubectl get pods -n chaos-mesh
```

なお、k3dクラスタの作成/削除だけを個別に行いたい場合は
`make cluster-create` / `make cluster-delete`(内部的に`infra/k3d/setup.sh` /
`infra/k3d/teardown.sh`を呼ぶ)も使える。

### 2. ロボット側(Gazebo + Nav2 + フェイルセーフ層)の起動

```bash
make ros2-up   # edge/docker/ の docker compose を起動

docker exec -d ros2_nav2_container bash -c \
  "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && \
   ros2 launch nav2_bringup_custom gazebo_sim.launch.py"

# 数秒待ってから
docker exec -d ros2_nav2_container bash -c \
  "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && \
   ros2 launch nav2_bringup_custom nav2_bringup.launch.py"
```

これで以下が全て起動する: Nav2フルスタック、`safety_bringup`(heartbeat_monitor,
safety_state_machine, watchdog, nav2_heartbeat_adapter, estop_bridge)。

安全状態の確認:

```bash
docker exec ros2_nav2_container bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic echo /safety/state"
```

### 3. ビルド・テスト

```bash
make ros2-build-test              # ワークスペース全体
make ros2-build-test PKG=watchdog # パッケージ単体(名前 or パスどちらでも可)
```

### 4. 可観測性(Prometheus + Grafana)

```bash
cd edge/docker && docker compose up -d prometheus grafana
```

Grafanaは`hacobot_health.json`ダッシュボードを自動プロビジョニングする
(データソース登録・ダッシュボードインポートとも手動操作不要)。`network_mode: host`の
ため、ホストのブラウザから`http://localhost:3001`(admin/admin)でアクセス可能。

### 5. カオス実験の実行

```bash
kubectl apply -f chaos/experiments/pod-chaos-kafka.yaml
kubectl get podchaos -n kafka

# フェイルセーフの自動検証(kubectlとROS2両方に触れるためホスト側で実行)
python3 chaos/test_scenarios/verify_failsafe_during_chaos.py --all --watch-sec 30
```

結果は`testing/eval-reports/chaos_test/*.json`に出力される。

### 6. 耐久試験

```bash
docker exec ros2_nav2_container bash -c \
  "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && \
   python3 /testing/soak_test/run_soak_test.py --duration 1h"
```

結果は`testing/soak_test/reports/[実行日時]/report.json`に出力される。

### 7. Kafka経由の本物のリモートE-Stop(ネイティブDocker Engine限定)

```bash
KAFKA_POD=$(kubectl get pods -n kafka -l strimzi.io/name=hacobot-kafka-kafka \
  -o jsonpath='{.items[0].metadata.name}')
echo "ESTOP" | kubectl exec -i -n kafka "$KAFKA_POD" -c kafka -- \
  bin/kafka-console-producer.sh --bootstrap-server localhost:9092 --topic robot-control
```

`estop_bridge`が実際にKafka経由でE-Stopを受信し、`/safety/state`が`SAFE_STOP`になる。

`demos/`配下には手順1〜7それぞれに対応する自動判定付きbashスクリプトも用意している
(`demos/README.md`参照。`bash demos/run_all.sh`でまとめて実行可能)。

## ドキュメント

- `docs/verification_guide.md`: 動作確認の手順(ステップごとの成功の目安付き)
- `docs/architecture-diagram.drawio`: ローカル検証環境の全体構成図
- `docs/aws-production-architecture.drawio`: マルチAZ EKSを想定したAWS本番運用構成図

## 既知の制約

- このリポジトリはWSL2上のdev/検証環境を前提としている。コンテナ実行基盤がDocker Desktopか
  ネイティブDocker Engine(`docker-ce`)かで、Kafka連携の一部(リモートE-Stopの実際の到達、
  カオス実験のロボット側への波及)が検証できるかどうかが変わる(Docker Desktopはコンテナを
  別ネットワーク名前空間のVMで動かすため、k3dクラスタ内のKafkaへ到達できない)。
  **注意**: `safety_bringup.launch.py`は`estop_bridge`を`assume_healthy: false`
  (実際にTCP疎通チェックを行う設定)で起動する。Docker Desktop環境ではこの疎通チェックが
  常に失敗し、`comm_bridge`のハートビートが一度も送られないため、起動から数秒で
  `heartbeat_monitor`がハートビート途絶を検知してSAFE_STOPへ遷移する
  (これはフェイルセーフとして正しい挙動だが、Docker Desktopで単に動作確認したいだけの場合は
  想定外の停止に見える)。回避するには`estop_bridge`の`assume_healthy`パラメータを
  `true`で上書きして起動すること
- k3dはシングルノード構成であり、クラスタ自体の高可用性は範囲外。マルチAZ構成のAWS本番設計は
  `docs/aws-production-architecture.drawio`を参照
- カオス実験実行中のロボット側の反応検知(`demos/05_chaos_experiment.sh`)は、Kafka Podの
  復旧速度(20〜50秒程度)と`estop_bridge`の接続チェック間隔(2秒)のタイミング次第で
  毎回検知できるとは限らない。フェイルセーフの実効性そのものは`demos/03`/`demos/07`で
  安定して確認できる
