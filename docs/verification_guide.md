# 動作確認ガイド

実際に手を動かして確認する順番で、ステップごとに「何が見えれば成功か」も含めてまとめる。
一番手軽な確認(ビルド)から、時間のかかる確認(耐久試験)の順に並べている。

前提: `docker`(WSL2上でDocker Engineが起動していること)、`kubectl`、`helm`、`helmfile`、
`k3d`が使える状態であること。Docker Desktopとネイティブ`docker-ce`の競合で
`/var/run/docker.sock`が不安定になる場合は、Docker Desktop側のWSL Integrationを
オフにするか、Docker Desktopを終了してからネイティブの`dockerd`
(`sudo systemctl restart docker`)を使うこと。

**重要**: **ネイティブDocker Engine**(Docker Desktopではない)を使うと、
`ros2_nav2_container`(`network_mode: host`)がWSL2ディストロ自身のネットワーク
名前空間を共有するため、k3dクラスタ内のKafkaへ実際に到達できるようになる
(`infra/k8s/kafka/kafka-cluster.yaml`のnodeportリスナー経由、`127.0.0.1:30092`)。
Docker Desktopでは別ネットワーク名前空間のため到達できない。このガイドの手順7は
ネイティブDocker Engineでのみ成立する。

## 1. ビルド・単体テスト(一番手軽、他の起動不要)

```bash
cd /home/gum/projects/hacobot
make up            # コンテナが無ければ起動
make build-test    # ワークスペース全体
```

**成功の目安**: `Summary: 9 packages finished` と
`Summary: 21 tests, 0 errors, 0 failures, 0 skipped`

## 2. Gazebo + Nav2 + フェイルセーフ層の起動確認

```bash
docker exec -d ros2_nav2_container bash -c \
  "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && \
   ros2 launch nav2_bringup_custom gazebo_sim.launch.py"
sleep 15
# GUIは重いので落としてOK(仕組みには影響しない)
docker exec ros2_nav2_container bash -c "ps aux | grep gzclient | awk '{print \$2}' | xargs -r kill -9"

docker exec -d ros2_nav2_container bash -c \
  "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && \
   ros2 launch nav2_bringup_custom nav2_bringup.launch.py"
sleep 30
docker exec ros2_nav2_container bash -c "source /opt/ros/humble/setup.bash && ros2 node list"
```

**成功の目安**: `/heartbeat_monitor` `/safety_state_machine` `/watchdog`
`/nav2_heartbeat_adapter` `/estop_bridge` `/bt_navigator` などが一覧に出る
(20〜30個程度)。

## 3. フェイルセーフの核心: 実際に壊してSAFE_STOPになるか

これが一番「動いている」ことを実感できる確認。

```bash
# 現在の安全状態を見る
docker exec ros2_nav2_container bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic echo /safety/state"
```

別ターミナルで:

```bash
# Nav2を意図的に殺す(component_container_isolatedがNav2本体)
docker exec ros2_nav2_container bash -c \
  "pkill -9 -f component_container_isolated"
```

**成功の目安**: 数秒以内に`/safety/state`が `NORMAL → DEGRADED → SAFE_STOP` と
変化するのがログに流れる。

復旧を試す場合:

```bash
docker exec ros2_nav2_container bash -c \
  "source /opt/ros/humble/setup.bash && \
   ros2 topic pub -1 /safety/recovery_command std_msgs/msg/Empty '{}'"
```

(2回送るとSAFE_STOP→MANUAL_RECOVERY→NORMAL。ただしNav2を殺したままだと、
根本原因未解決のため再びSAFE_STOPへ戻るのが正しい挙動)

## 4. 可観測性(Prometheus/Grafana)

```bash
cd edge/docker && docker compose up -d prometheus grafana
```

ブラウザで `http://localhost:3001` (admin/admin) を開き、
左メニュー→Dashboards→`hacobot`フォルダ→`hacobot System Health`。

**成功の目安**: 一番上の"Safety State"パネルが現在の状態(手順3で変化させたなら
反映されているはず)を色付きで表示する。

## 5. カオス実験(k3d側)

```bash
bash infra/deploy.sh   # まだ構築していなければ(Kafka/SeaweedFS)
export PATH="$HOME/.local/bin:$PATH"
helmfile -f infra/helmfile.yaml sync   # Chaos Mesh込みで同期

kubectl apply -f chaos/experiments/pod-chaos-kafka.yaml
kubectl get pods -n kafka   # KafkaのPodのAGEが一瞬0になって再起動するのが見える
```

**注意**: ネイティブDocker Engineであれば、Kafka Podが落ちている間に
`estop_bridge`のログに`Kafka connectivity ... changed: reachable -> unreachable`が
出ることがある(ロボット側への波及の入り口)。ただしKafkaの復旧(数十秒)と
接続チェック間隔(2秒)のタイミング次第で見逃すこともある(タイミングが合わなければ
警告が出るだけで、それ自体は失敗ではない)。Docker Desktopの場合はそもそも
到達できないため観測できない。ロボット側フェイルセーフの実効性を安定して
確認したい場合は手順3または7を使うこと。

## 6. 耐久試験スクリプト(短時間で試す場合)

```bash
docker exec ros2_nav2_container bash -c \
  "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && \
   python3 /testing/soak_test/run_soak_test.py --duration 60s --sample-interval 15s"
```

**成功の目安**: `testing/soak_test/reports/[日時]/report.json` が生成され、
`process_memory`などが埋まっている。

## 7. Kafka経由の本物のリモートE-Stop(ネイティブDocker Engine限定)

```bash
KAFKA_POD=$(kubectl get pods -n kafka -l strimzi.io/name=hacobot-kafka-kafka \
  -o jsonpath='{.items[0].metadata.name}')
echo "ESTOP" | kubectl exec -i -n kafka "$KAFKA_POD" -c kafka -- \
  bin/kafka-console-producer.sh --bootstrap-server localhost:9092 --topic robot-control

docker exec ros2_nav2_container bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic echo /safety/state"
```

**成功の目安**: `estop_bridge`のログに
`E-Stop command received: ESTOP` →
`CRITICAL anomaly from 'estop_bridge': ... -> immediate SAFE_STOP` が出て、
`/safety/state`が`SAFE_STOP`になる。リモートE-Stop受信経路を、本物のKafkaメッセージで
エンドツーエンドに確認できる手順。
