# nav2_bringup_custom

TurtleBot3(Waffle)のGazeboシミュレーションと、Nav2 + SLAM Toolboxによる自律走行スタックを
統合するbringupパッケージ。既存のオープンパッケージ(`turtlebot3_gazebo`, `nav2_bringup`,
`slam_toolbox`)を薄くラップしているだけで、ワールドやNav2本体は再実装していない。

事前に作った地図とAMCLで自己位置推定する構成ではなく、**走行しながら地図を作るオンライン
SLAM(slam_toolbox)を自己位置推定源として使う**構成になっている(`nav2_params.yaml`参照)。

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

ターミナル1: Gazeboシミュレータを起動

```bash
docker exec -it ros2_nav2_container bash -c "
    source /opt/ros/jazzy/setup.bash && source install/setup.bash &&
    ros2 launch nav2_bringup_custom gazebo_sim.launch.py
"
```

グリッド(ワールド)が表示されるまで、初回のみ3Dモデルのダウンロードで1〜2分かかることがある。

ターミナル2: Nav2 + SLAM Toolboxスタックを起動

```bash
docker exec -it ros2_nav2_container bash -c "
    source /opt/ros/jazzy/setup.bash && source install/setup.bash &&
    ros2 launch nav2_bringup_custom nav2_bringup.launch.py
"
```

ターミナル3(任意): RVizで地図生成・ゴール指定を行う場合

```bash
docker exec -it ros2_nav2_container bash -c "
    source /opt/ros/jazzy/setup.bash && source install/setup.bash &&
    ros2 launch nav2_bringup rviz_launch.py
"
```

RViz上で「Nav2 Goal」を指定すると、SLAM Toolboxが生成中の地図上をNav2が自律走行する。

ターミナル4(任意): キーボードで操作する場合

```bash
docker exec -it ros2_nav2_container bash -c "
    source /opt/ros/jazzy/setup.bash && \
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
"
```

## 構成ファイル

- `launch/gazebo_sim.launch.py`: `turtlebot3_gazebo` の `turtlebot3_world.launch.py` を起動するだけの薄いラッパー
- `launch/nav2_bringup.launch.py`: `nav2_bringup` の `bringup_launch.py` を `slam:=true` で起動するだけの薄いラッパー
- `params/nav2_params.yaml`: TurtleBot3 Waffle向けにチューニングされたNav2パラメータ + SLAM Toolboxパラメータ(由来は`params/nav2_params.yaml`冒頭コメント参照)
