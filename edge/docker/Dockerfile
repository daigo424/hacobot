FROM osrf/ros:jazzy-desktop

# TurtleBot3シミュレーション(Gazebo Harmonic/gz sim) + Nav2 + SLAM Toolbox一式。
# ros-jazzy-turtlebot3-gazeboはまだバイナリ配布が無いため、ソースをedge/ros2_ws/src/に
# vendorしてworkspace全体のcolcon buildでビルドする(このDockerfile内では入れない)。
# 理由: docker-compose.ymlが../ros2_ws:/workspaceを丸ごとbind mountしているため、
# イメージビルド時に/workspace/src以下へ焼き込んでも実行時のmountで隠れてしまう。
RUN apt-get update && apt-get install -y --no-install-recommends \
    x11-apps \
    git \
    ros-jazzy-ros-gz \
    ros-jazzy-turtlebot3 \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-teleop-twist-keyboard \
    python3-colcon-common-extensions \
    python3-rosdep \
    librdkafka-dev \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# turtlebot3_gazebo(ros_gz_sim版)・explore_lite(m-explore-ros2)はvendorしたソースパッケージ
# なので、その依存関係をrosdepで解決できるようにしておく(apt-get installでの手動列挙より
# vendor元のpackage.xmlに追従できて壊れにくい)。実際の解決・インストールは
# workspace側でrosdep install --from-paths src --ignore-src する運用を想定。
RUN rosdep update

ENV TURTLEBOT3_MODEL=waffle
ENV QT_X11_NO_MITSHM=1

WORKDIR /workspace
