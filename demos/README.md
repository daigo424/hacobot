# demos/

`docs/verification_guide.md`の手順1〜7を、それぞれ実行・自動判定できるbashスクリプトに
したもの。各スクリプトは対応する手順を実行し、最後に`[PASS]`/`[FAIL]`で結果を表示する。

## 使い方

```bash
# 1つずつ実行
bash demos/01_build_test.sh
bash demos/02_launch_stack.sh
bash demos/03_break_and_verify_failsafe.sh
bash demos/04_observability.sh
bash demos/05_chaos_experiment.sh
bash demos/06_soak_test.sh
bash demos/07_real_kafka_estop.sh

# まとめて実行(1つ失敗したら停止)
bash demos/run_all.sh

# 一部だけ実行(例: 手順1〜3のみ)
bash demos/run_all.sh 1 2 3
```

## 対応表

| スクリプト | `docs/verification_guide.md`の手順 | 前提 |
|---|---|---|
| `01_build_test.sh` | 1. ビルド・単体テスト | Dockerが使えること |
| `02_launch_stack.sh` | 2. Gazebo+Nav2+フェイルセーフ層の起動確認 | `01`でイメージがビルド済みであること |
| `03_break_and_verify_failsafe.sh` | 3. 実際に壊してSAFE_STOPになるか | `02`でスタックが起動済みであること |
| `04_observability.sh` | 4. Prometheus/Grafana | `02`でスタックが起動済みであること |
| `05_chaos_experiment.sh` | 5. カオス実験(k3d側) | `bash infra/deploy.sh`と`helmfile sync`(Chaos Mesh)が完了していること |
| `06_soak_test.sh` | 6. 耐久試験スクリプトのドライラン | `02`でスタックが起動済みであること |
| `07_real_kafka_estop.sh` | 7. Kafka経由の本物のリモートE-Stop | `02`起動済み + `bash infra/deploy.sh`完了 + **ネイティブDocker Engine** |

`05`/`07`は、k3dクラスタ(`infra/`)側の別セットアップが前提になる点に注意
(`01`〜`04`, `06`は`edge/docker/`のROS2コンテナだけで完結する)。

## ネイティブDocker Engine vs Docker Desktop

`07`(および`05`の「ロボット側への波及」確認部分)は**ネイティブDocker Engine**
(WSL2に直接`docker-ce`を入れたもの)でのみ成立する。Docker Desktopを使っている場合、
`ros2_nav2_container`(`network_mode: host`)はDocker Desktop VM側のネットワーク
名前空間を共有するため、k3dクラスタ内のKafkaへ到達できず、`estop_bridge`の
Kafka疎通は常に失敗する(`assume_healthy`バイパスが必要になる)。

## 環境変数での調整

- `demos/03_break_and_verify_failsafe.sh`: `WATCH_SEC`(既定20秒)でSAFE_STOP遷移の監視時間を変更
- `demos/06_soak_test.sh`: `DURATION`(既定60s)/`SAMPLE_INTERVAL`(既定15s)で試験時間を変更
  (例: `DURATION=24h SAMPLE_INTERVAL=5m bash demos/06_soak_test.sh` で本番相当の耐久試験も実行可能)

## 注意

- `05_chaos_experiment.sh`はネイティブDocker Engineであればロボット側への波及
  (`estop_bridge`の接続断検知ログ)も確認を試みるが、Kafka Podの復旧速度と
  接続チェック間隔(2秒)のタイミング次第で検知できないことがある(その場合は
  警告が出るだけで、実験自体の失敗ではない)。フェイルセーフの実効性を安定して
  確認したいときは`03_break_and_verify_failsafe.sh`または`07_real_kafka_estop.sh`を使うこと。
- 各スクリプトは`docker exec`でコンテナ内のROS2/kubectlコマンドを呼ぶため、
  `ros2_nav2_container`が起動していない状態で`02`以降を実行するとエラーで案内が出る。
