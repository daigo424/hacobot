# 素のk3dクラスタだけ欲しい場合(runs/配下の軽量な動作確認スクリプトなど)は
# infra/k3d/setup.sh・teardown.sh を直接呼ぶこと(CLUSTER_NAME環境変数で名前変更可)。
# フルスタック(Kafka/SeaweedFS/Chaos Mesh)が要る場合は下記のinfra-deploy/infra-destroyを使う。

.PHONY: infra-deploy infra-destroy

# infra-deploy: k3dクラスタ構築 + SeaweedFS/Kafka/Chaos Meshのデプロイ(infra/deploy.sh)
# infra-destroy: 上記のうちKafka/Helmリリースだけを後片付け(k3dクラスタ自体は残す)。
#                クラスタごと消したい場合は infra/k3d/teardown.sh を直接使うこと。
infra-deploy:
	bash infra/deploy.sh

infra-destroy:
	bash infra/destroy.sh

# S3互換ゲートウェイへport-forwardする(認証情報はinfra/helm-values/seaweedfs-values.yaml参照。
# PoC用の固定値でありSecret化していないため本番投入前は必ず差し替えること)。
.PHONY: seaweedfs-ui
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

.PHONY: up build test build-test new-launch-pkg rviz

COMPOSE_PJ_NAME    := hacobot
COMPOSE            := docker compose -f edge/docker/docker-compose.yml -p $(COMPOSE_PJ_NAME)
RUN                := $(COMPOSE) run --rm --remove-orphans
EXEC               := $(COMPOSE) exec
ROS2_SERVICE       := ros2-nav2
ROS2_CONTAINER     := ros2_nav2_container
ROS2_WS            := /workspace
PKG                ?=
PKG_NAME           := $(if $(PKG),$(notdir $(PKG)),)
COLCON_SELECT      := $(if $(PKG_NAME),--packages-select $(PKG_NAME),)
CMD_ROS2_SOURCE    := source /opt/ros/jazzy/setup.bash
CMD_ROS2_WS_SOURCE := source $(ROS2_WS)/install/setup.bash

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

# launchファイルだけを持つ新しいbringupパッケージ(nav2_bringup_custom, safety_bringupと
# 同種)を対話的に作成する(scripts/create_launch_package.py)。colcon build後、他のlaunch
# ファイルからIncludeLaunchDescriptionで呼び出せる状態まで生成する。
# 対話入力が要るためホストのpython3で直接実行する(コンテナ経由にしない)。
new-launch-pkg:
	python3 scripts/create_launch_package.py

# spawn_robotが起動したロボット(既定はROBOT_ID=tb3_01)の地図/コストマップ/LaserScan/
# RobotModelを表示するRViz設定で起動する
# (edge/ros2_ws/src/nav2_bringup_custom/rviz/hacobot_view.rviz)。
# 各ロボットは専用の/tb3_0N/tfを持つ(ロボットごとに完全独立させる設計。詳細は
# spawn_robot.launch.pyのdocstring参照)ため、RViz自身のプロセスにも
# /tf:=/$(ROBOT_ID)/tfのremapが必要(無いとRVizはグローバルな/tfしか見ず何も表示されない)。
# hacobot_view.rviz内の全トピックは"tb3_01"という文字列だけで一貫して書かれているため、
# sedで一括置換すれば任意のROBOT_IDに対応できる(ファイルは複製しない)。
# 例: make rviz ROBOT_ID=tb3_02
ROBOT_ID ?= tb3_01

rviz:
	$(EXEC) $(ROS2_SERVICE) bash -c "\
		$(CMD_ROS2_SOURCE) && \
		$(CMD_ROS2_WS_SOURCE) && \
		RVIZ_SRC=\$$(ros2 pkg prefix nav2_bringup_custom)/share/nav2_bringup_custom/rviz/hacobot_view.rviz && \
		RVIZ_TMP=/tmp/hacobot_view_$(ROBOT_ID).rviz && \
		sed 's/tb3_01/$(ROBOT_ID)/g' \$$RVIZ_SRC > \$$RVIZ_TMP && \
		rviz2 -d \$$RVIZ_TMP \
			--ros-args -r /tf:=/$(ROBOT_ID)/tf -r /tf_static:=/$(ROBOT_ID)/tf_static \
			-r __ns:=/$(ROBOT_ID)"

login:
	$(EXEC) $(ROS2_SERVICE) bash -c \
	  "$(CMD_ROS2_SOURCE) && $(CMD_ROS2_WS_SOURCE) && bash"

colcon:
	$(EXEC) $(ROS2_SERVICE) bash -c \
	  "$(CMD_ROS2_SOURCE) && $(CMD_ROS2_WS_SOURCE) && \
	   cd $(ROS2_WS) && $(CMD_RUN)"

colcon-build:
	$(MAKE) colcon CMD_RUN="colcon build $(COLCON_SELECT)"
colcon-test:
	# --executor sequential: 複数パッケージを並列実行すると、別パッケージのgtest同士が
	# 同じROSトピック名(例: /safety/anomaly_event)で混信することがあるため直列実行する
	$(MAKE) colcon CMD_RUN="colcon test --executor sequential $(COLCON_SELECT) && colcon test-result --verbose"
colcon-build-test: colcon-build colcon-test

topic-list:
	$(MAKE) colcon CMD_RUN="ros2 topic list"
