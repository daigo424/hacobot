CLUSTER_NAME ?= hacobot-cluster

.PHONY: cluster-create cluster-delete

# k3dクラスタの実際の構築/削除ロジックは infra/k3d/setup.sh・teardown.sh に一本化している
# (Makefile側に引数を重複定義すると、どちらかだけ更新して片方が古いまま残るため)。
cluster-create:
	CLUSTER_NAME=$(CLUSTER_NAME) bash infra/k3d/setup.sh

cluster-delete:
	CLUSTER_NAME=$(CLUSTER_NAME) bash infra/k3d/teardown.sh

# --- ROS2ワークスペース (edge/ros2_ws) ---
#
# ビルド/テストは edge/docker/ の ros2_nav2_container 内で実行する。
# PKGはパッケージ名でもホスト側のパス(例: edge/ros2_ws/src/safety/heartbeat_monitor)
# でもどちらでも指定可能($(notdir ...)でパスの最後の要素=パッケージ名だけを取り出す)。
# PKGを省略するとワークスペース全体が対象になる。
#
# 例:
#   make ros2-build PKG=heartbeat_monitor
#   make ros2-test  PKG=edge/ros2_ws/src/safety/watchdog
#   make ros2-build-test PKG=safety_state_machine
#   make ros2-build   # 全パッケージ

.PHONY: ros2-up ros2-build ros2-test ros2-build-test

ROS2_CONTAINER ?= ros2_nav2_container
ROS2_WS        := /workspace
PKG            ?=
PKG_NAME       := $(if $(PKG),$(notdir $(PKG)),)
COLCON_SELECT  := $(if $(PKG_NAME),--packages-select $(PKG_NAME),)

ros2-up:
	cd edge/docker && docker compose up -d

ros2-down:
	cd edge/docker && docker compose down

ros2-build:
	docker exec $(ROS2_CONTAINER) bash -c "\
		source /opt/ros/humble/setup.bash && \
		( [ -f $(ROS2_WS)/install/setup.bash ] && source $(ROS2_WS)/install/setup.bash || true ) && \
		cd $(ROS2_WS) && \
		colcon build $(COLCON_SELECT)"

ros2-test:
	# --executor sequential: 複数パッケージを並列実行すると、別パッケージのgtest同士が
	# 同じROSトピック名(例: /safety/anomaly_event)で混信することがあるため直列実行する
	docker exec $(ROS2_CONTAINER) bash -c "\
		source /opt/ros/humble/setup.bash && \
		source $(ROS2_WS)/install/setup.bash && \
		cd $(ROS2_WS) && \
		colcon test --executor sequential $(COLCON_SELECT) && \
		colcon test-result --verbose"

ros2-build-test: ros2-build ros2-test

