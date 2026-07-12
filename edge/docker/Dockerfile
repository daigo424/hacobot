FROM osrf/ros:humble-desktop

# TurtleBot3シミュレーション + Nav2 + SLAM Toolbox一式。
# runs/gazebo.sh は起動の度にapt-get installしていたが、ここではイメージビルド時に
# 一度だけ入れることで起動を速く・再現可能にする。
RUN apt-get update && apt-get install -y --no-install-recommends \
    x11-apps \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3-simulations \
    ros-humble-turtlebot3 \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-teleop-twist-keyboard \
    python3-colcon-common-extensions \
    librdkafka-dev \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# 補足1: Gazebo公式のモデルサーバーURL変更によるハング・タイムアウトを回避する設定。
# (runs/gazebo.sh で毎回ランタイムパッチしていたのをビルド時に焼き込む)
RUN mkdir -p /root/.gazebo/config && \
    printf 'url: https://fuel.gazebosim.org\nversion: 1.0\n' > /root/.gazebo/config/config.yaml

# 補足2: spawn_entity.py のデフォルト待機時間(30秒)は、モデルの初回ダウンロードで
# 超過してワールドだけ表示されロボットが出現しない問題を起こす(cold cache時に実測143秒)。
# 180秒に延長する。(runs/gazebo.sh の起動時パッチと同じ内容をビルド時に焼き込む)
# (アンカー$は使わない: このファイルはCRLF改行が混在しており$の直前にマッチしないため)
RUN sed -i "s/'-z', '0.01'/'-z', '0.01', '-timeout', '180'/" \
    /opt/ros/humble/share/turtlebot3_gazebo/launch/spawn_turtlebot3.launch.py

ENV TURTLEBOT3_MODEL=waffle
ENV QT_X11_NO_MITSHM=1

WORKDIR /workspace
