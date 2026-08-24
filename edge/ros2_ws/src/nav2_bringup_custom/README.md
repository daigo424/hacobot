# nav2_bringup_custom

TurtleBot3(Waffle)のGazeboシミュレーションと、Nav2 + SLAM Toolboxによる自律走行スタックを
統合するbringupパッケージ。既存のオープンパッケージ(`turtlebot3_gazebo`, `nav2_bringup`,
`slam_toolbox`)を薄くラップしているだけで、ワールドやNav2本体は再実装していない。

事前に作った地図とAMCLで自己位置推定する構成ではなく、**走行しながら地図を作るオンライン
SLAM(slam_toolbox)を自己位置推定源として使う**構成になっている
(`nav2_params_builder`パッケージ参照)。

**注意**: このパッケージの`nav2_bringup.launch.py`は単体動作確認用の経路で、実際の開発・検証は
マルチロボット対応の`spawn_robot`/`create_world`パッケージ経由(`make create-world` →
`make build-map-auto`、リポジトリ直下の`README.md`参照)で行っている。
Gazebo(TurtleBot3本家の`turtlebot3_world.launch.py`)自体はGazebo Harmonic上でも`/scan`等の
ブリッジを含め正常動作する。GUI("gz sim -g")を`kill -9`する等、外部からの終了は避けること
(本家launchが`on_exit_shutdown=true`を設定しており、GUIの終了がサーバー・ブリッジまで
巻き添えでシャットダウンさせる)。

## セットアップ

初回のみ、Dockerイメージをビルドしてコンテナを起動する。

```bash
cd edge/docker
docker compose up -d
```

コンテナ内でワークスペースをビルドする。

```bash
docker exec -it ros2_nav2_container bash -c "
    source /opt/ros/jazzy/setup.bash &&
    colcon build --packages-select nav2_bringup_custom &&
    source install/setup.bash
"
```

## 起動方法

`nav2_bringup.launch.py`1本でGazebo・Nav2+SLAM Toolboxスタック・RViz(`rviz/view.rviz`)・
`safety_bringup`一式をまとめて起動する。

```bash
docker exec -it ros2_nav2_container bash -c "
    source /opt/ros/jazzy/setup.bash && source install/setup.bash &&
    ros2 launch nav2_bringup_custom nav2_bringup.launch.py
"
```

グリッド(ワールド)が表示されるまで、初回のみ3Dモデルのダウンロードで1〜2分かかることがある。
RViz上で「Nav2 Goal」を指定すると、SLAM Toolboxが生成中の地図上をNav2が自律走行する。

ターミナル2(任意): キーボードで操作する場合

```bash
docker exec -it ros2_nav2_container bash -c "
    source /opt/ros/jazzy/setup.bash && \
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
"
```

## 構成ファイル

- `launch/nav2_bringup.launch.py`: `turtlebot3_gazebo`の`turtlebot3_world.launch.py`(Gazebo)、
  `nav2_bringup`の`bringup_launch.py`(`slam:=true`)、RViz、`safety_bringup`をまとめて起動する
- `params/defaults/`: 本家Nav2(`nav2_params.yaml`、MPPIベース)・TurtleBot3 Waffleサンプル
  (`waffle.yaml`)・slam_toolbox本家(`mapper_params_online_sync.yaml`)の無加工リファレンス。
  実際にNav2へ渡すパラメータは、これらを`nav2_params_builder`パッケージ
  (`nav2_params_builder/builder.py`)が実行時にマージ・上書きして生成する(車体依存値は
  `waffle.yaml`から読み込み、`collision_monitor`の安全ゾーンや`docking_server`のダミー実装等
  hacobot固有の差分だけをコード上で明示的に上書きする方針。詳細は`build_nav2_params_yaml()`の
  docstring参照)。以前あった単一マージ済みファイル`params/nav2_params.yaml`は、本家からの差分・
  非差分が区別できなかったためこの構成に置き換えて削除した
