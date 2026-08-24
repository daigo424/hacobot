FROM osrf/ros:jazzy-desktop

RUN apt -y update
RUN apt-get install -y x11-apps

# TurtleBot3シミュレーション(Gazebo Harmonic/gz sim) + Nav2 + SLAM Toolbox一式。
# ros-jazzy-turtlebot3-gazeboはまだバイナリ配布が無いため、ソースをedge/ros2_ws/src/に
# vendorしてworkspace全体のcolcon buildでビルドする(このDockerfile内では入れない)。
# 理由: docker-compose.ymlが../ros2_ws:/workspaceを丸ごとbind mountしているため、
# イメージビルド時に/workspace/src以下へ焼き込んでも実行時のmountで隠れてしまう。
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-gz-ros2-control* \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-turtlebot3 \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rqt-tf-tree \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rviz-visual-tools \
    python3-colcon-common-extensions \
    python3-colcon-argcomplete \
    python3-rosdep \
    librdkafka-dev \
    pkg-config \
    git \
    bear \
    vim \
    && rm -rf /var/lib/apt/lists/*


ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN rosdep update

# apt版joint_trajectory_controller(4.40.1)は同じくapt版のcontroller_interface(4.45.2)とABI不整合でon_init()がSEGVする
# (packages.ros.org側の既知の不整合。ros-controls/ros2_control#2049)。ソースビルドで現行のcontroller_interfaceに合わせる。
RUN git clone --branch ${ROS_DISTRO} --depth 1 \
      https://github.com/ros-controls/ros2_controllers.git \
      /opt/ros2_controllers_ws/src/ros2_controllers

RUN apt-get update && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --from-paths /opt/ros2_controllers_ws/src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd /opt/ros2_controllers_ws && \
    colcon build --merge-install

# bash -c "... && bash"のような入れ子bash越しだとcomplete登録(colcon-argcomplete)が
# 子プロセスに引き継がれない(exportされる環境変数と違い、complete -Fはシェルのメモリ内状態のため)。
# 対話シェル自身の.bashrcで毎回sourceすることで、どの経路で入っても補完が効くようにする。
RUN grep -qxF '# --- ros2 shell setup (learn-hw) ---' /root/.bashrc || \
    cat >> /root/.bashrc <<EOF
# --- ros2 shell setup (learn-hw) ---
source /opt/ros/${ROS_DISTRO}/setup.bash
source /opt/ros2_controllers_ws/install/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
test -f /workspace/install/setup.bash && source /workspace/install/setup.bash
EOF

ENV TURTLEBOT3_MODEL=waffle
ENV QT_X11_NO_MITSHM=1

WORKDIR /workspace
