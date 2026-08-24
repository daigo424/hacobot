# infra-deploy: k3dクラスタ構築 + SeaweedFS/Kafka/Chaos Meshのデプロイ(infra/deploy.sh)
# infra-destroy: 上記のうちKafka/Helmリリースだけを後片付け(k3dクラスタ自体は残す)。
#                クラスタごと消したい場合は infra/k3d/teardown.sh を直接使うこと。
infra-deploy:
	bash infra/deploy.sh

infra-destroy:
	bash infra/destroy.sh

# S3互換ゲートウェイへport-forwardする(認証情報はinfra/helm-values/seaweedfs-values.yaml参照。
# PoC用の固定値でありSecret化していないため本番投入前は必ず差し替えること)。
seaweedfs-ui:
	@echo -----------------------------
	@echo "SeaweedFS S3 API: http://localhost:8333"
	@echo "Access Key: hacobot-admin"
	@echo "Secret Key: hacobot-admin-secret"
	@echo -----------------------------
	@echo "Port-forward starting... Ctrl+C to stop"
	kubectl port-forward -n seaweedfs svc/seaweedfs-s3 8333:8333

# --- ROS2ワークスペース (edge/ros2_ws) ---
#
# ビルド/テストは edge/docker/ の ros2_nav2_container 内で実行する。
# PKGはパッケージ名でもホスト側のパス(例: edge/ros2_ws/src/safety/heartbeat_monitor)
# でもどちらでも指定可能($(notdir ...)でパスの最後の要素=パッケージ名だけを取り出す)。
# PKGを省略するとワークスペース全体が対象になる。
#
# 例:
#   make colcon-build PKG=heartbeat_monitor
#   make colcon-test  PKG=edge/ros2_ws/src/safety/watchdog
#   make colcon-build-test PKG=safety_state_machine
#   make colcon-build   # 全パッケージ
COMPOSE_PJ_NAME    := hacobot
# WSL2かネイティブLinuxかでGPUパススルーの構成が別物になる(edge/docker/docker-compose.gpu-*.yml参照)
# ため、/proc/versionで自動判定してoverrideファイルを差し替える。
ifneq ($(shell grep -qi microsoft /proc/version 2>/dev/null && echo wsl),)
GPU_COMPOSE_FILE   := edge/docker/docker-compose.gpu-wsl.yml
else
GPU_COMPOSE_FILE   := edge/docker/docker-compose.gpu-native.yml
endif
COMPOSE            := docker compose -f edge/docker/docker-compose.yml -f $(GPU_COMPOSE_FILE) -p $(COMPOSE_PJ_NAME)
RUN                := $(COMPOSE) run --rm --remove-orphans
EXEC               := $(COMPOSE) exec
ROS2_SERVICE       := ros2-nav2
ROS2_CONTAINER     := ros2_nav2_container
ROS2_WS            := /workspace
PKG                ?=
PKG_NAME           := $(if $(PKG),$(notdir $(PKG)),)
COLCON_SELECT      := $(if $(PKG_NAME),--packages-select $(PKG_NAME),)
CMD_ROS2_SOURCE    := source /opt/ros/jazzy/setup.bash && source /opt/ros2_controllers_ws/install/setup.bash && source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
CMD_ROS2_WS_SOURCE := test -f $(ROS2_WS)/install/setup.bash && source $(ROS2_WS)/install/setup.bash || true


up:
	@test -f .env || cp .env.example .env
	$(COMPOSE) --env-file .env up -d
	@. ./.env 2>/dev/null; \
	echo -----------------------------; \
	echo "Grafana: http://localhost:$${GRAFANA_PORT:-3001}"; \
	echo "  User:     $${GRAFANA_ADMIN_USER:-admin}"; \
	echo "  Password: $${GRAFANA_ADMIN_PASSWORD:-admin}"; \
	echo -----------------------------

down:
	$(COMPOSE) --env-file .env down

build:
	$(COMPOSE) --env-file .env build --no-cache

login:
	$(EXEC) $(ROS2_SERVICE) bash

# ros2 launchで起動したプロセスをCtrl+Cせず再起動すると、gz sim(rubyラッパー)や
# ros_gz_bridge系ノードが孤児化して残り続けることがある(SIGINTがlaunchの子孫全員に
# 伝播しないケース)。それらをまとめてSIGKILLで掃除する。
# docker-compose.ymlのpid: hostによりホストとPID空間を共有しているため、
# 1) -u rootでコンテナ内(root実行)由来のものだけに絞り、ホスト側の一般ユーザープロセス
#    (make/docker compose exec自体など)を誤って巻き込まない
# 2) パターン中の各キーワードを[x]で1文字だけ字句分割し、pkill自身のコマンドライン
#    (-fの引数にこのパターン文字列がそのまま載る)に自己マッチしてpkillが自爆するのを防ぐ
KILL_ROS_PATTERN := ([r]os2|g[z] sim|gzs[e]rver|gzcl[i]ent|ros[_]gz|rvi[z]2|robot_state_publ[i]sher|[n]av2|component_conta[i]ner|teleop_twist_key[b]oard|rub[y].*gz_tools)
kill-ros:
	$(EXEC) $(ROS2_SERVICE) bash -c \
	  "pkill -9 -u root -f '$(KILL_ROS_PATTERN)' || true"

colcon:
	$(EXEC) $(ROS2_SERVICE) bash -c \
	  "$(CMD_ROS2_SOURCE) && $(CMD_ROS2_WS_SOURCE) && \
	   cd $(ROS2_WS) && $(CMD_RUN)"

# bear(edge/workspace/.clangdが参照する/workspace/compile_commands.jsonを生成)。
# --appendが無いと--packages-select時に他パッケージ分が上書きで消えるため必須。
# CLEAN=1でbuild/install/log/compile_commands.jsonを削除してからビルドする。
COLCON_BUILD_CLEAN := $(if $(CLEAN),rm -rf build/ install/ log/ compile_commands.json &&,)
COLCON_BUILD_WARN := -DCMAKE_WARN_DEPRECATED=$(if $(WARN),ON,OFF)
colcon-build:
	$(MAKE) colcon CMD_RUN="$(COLCON_BUILD_CLEAN) bear --append -- colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON $(COLCON_BUILD_WARN) --no-warn-unused-cli $(COLCON_SELECT)"
colcon-build-clean:
	$(MAKE) colcon-build CLEAN=1
colcon-build-clean-warn:
	$(MAKE) colcon-build CLEAN=1 WARN=1
colcon-test:
	# --executor sequential: 複数パッケージを並列実行すると、別パッケージのgtest同士が
	# 同じROSトピック名(例: /safety/anomaly_event)で混信することがあるため直列実行する
	$(MAKE) colcon CMD_RUN="colcon test --executor sequential $(COLCON_SELECT) && colcon test-result --verbose"
colcon-build-test: colcon-build colcon-test

install-packages:
	$(MAKE) colcon CMD_RUN="apt-get update && rosdep install --from-paths src --ignore-src -r -y"

launch-urdf-display:
	$(MAKE) colcon CMD_RUN="ros2 launch urdf_tutorial display.launch.py model:=/workspace/src/my_robot_description/urdf/$(FILENAME)"

rqt-graph:
	$(MAKE) colcon CMD_RUN="rqt_graph"

# ROS2: デバッグ用の便利コマンド群
topic-list:
	$(MAKE) colcon CMD_RUN="ros2 topic list"
tf2-tools-view-frames:
	$(MAKE) colcon CMD_RUN="ros2 run tf2_tools view_frames --ros-args --remap /tf:=/tb3_builder/tf --remap /tf_static:=/tb3_builder/tf_static"
rqt-tf-tree:
	$(MAKE) colcon CMD_RUN="ros2 run rqt_tf_tree rqt_tf_tree --ros-args -p use_sim_time:=true -r /tf:=/tb3_builder/tf -r /tf_static:=/tb3_builder/tf_static"
topic-hz-tf:
	$(MAKE) colcon CMD_RUN="ros2 topic hz /tb3_builder/tf"
topic-hz-tf-none:
	$(MAKE) colcon CMD_RUN="ros2 topic hz /tf"
topic-hz-tf-static:
	$(MAKE) colcon CMD_RUN="ros2 topic hz /tb3_builder/tf_static"
topic-hz-tf-static-none:
	$(MAKE) colcon CMD_RUN="ros2 topic hz /tf_static"
topic-echo-tf-static:
	$(MAKE) colcon CMD_RUN="ros2 topic echo /tb3_builder/tf_static --qos-durability transient_local --qos-reliability reliable --use-sim-time --once"
topic-echo-tf-static-none:
	$(MAKE) colcon CMD_RUN="ros2 topic echo /tf_static --qos-durability transient_local --qos-reliability reliable --use-sim-time --once"
param-dump-slam-toolbox:
	$(MAKE) colcon CMD_RUN="ros2 param dump /tb3_builder/slam_toolbox"
lifecycle-get-slam-toolbox:
	$(MAKE) colcon CMD_RUN="ros2 lifecycle get /tb3_builder/slam_toolbox"
param-get-distance-variance-penalty:
	$(MAKE) colcon CMD_RUN="ros2 param get /tb3_builder/slam_toolbox distance_variance_penalty"
param-get-angle-variance-penalty:
	$(MAKE) colcon CMD_RUN="ros2 param get /tb3_builder/slam_toolbox angle_variance_penalty"

# build_map.launch.pyのexploration_mode:=manual時はteleop_twist_keyboardを別ターミナルで
# 起動する必要がある(tty生読み取りのためros2 launch配下のノードにはできない、
# build_map.launch.pyのdocstring参照)。VSCodeのlaunch.jsonから個別に選んで起動できるよう、
# create-world/build-map-auto/build-map-manual/teleop/save-mapをそれぞれ独立したターゲットにする。
create-world:
	$(MAKE) colcon CMD_RUN="ros2 run create_world world_supervisor.py"
build-map-auto:
	$(MAKE) colcon CMD_RUN="ros2 launch spawn_robot build_map.launch.py exploration_mode:=auto"
build-map-manual:
	$(MAKE) colcon CMD_RUN="ros2 launch spawn_robot build_map.launch.py exploration_mode:=manual"
teleop:
	$(MAKE) colcon CMD_RUN="ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=/tb3_builder/cmd_vel_nav2_raw"
save-map:
	$(MAKE) colcon CMD_RUN="ros2 topic pub --once /tb3_builder/save_map_trigger std_msgs/msg/Empty {}"
