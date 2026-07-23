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
#   make build PKG=heartbeat_monitor
#   make test  PKG=edge/ros2_ws/src/safety/watchdog
#   make build-test PKG=safety_state_machine
#   make build   # 全パッケージ

.PHONY: up build test build-test

ROS2_CONTAINER ?= ros2_nav2_container
ROS2_WS        := /workspace
PKG            ?=
PKG_NAME       := $(if $(PKG),$(notdir $(PKG)),)
COLCON_SELECT  := $(if $(PKG_NAME),--packages-select $(PKG_NAME),)

up:
	@test -f .env || cp .env.example .env
	docker compose -f edge/docker/docker-compose.yml --env-file .env up -d
	@. ./.env 2>/dev/null; \
	echo -----------------------------; \
	echo "Grafana: http://localhost:$${GRAFANA_PORT:-3001}"; \
	echo "  User:     $${GRAFANA_ADMIN_USER:-admin}"; \
	echo "  Password: $${GRAFANA_ADMIN_PASSWORD:-admin}"; \
	echo -----------------------------

down:
	docker compose -f edge/docker/docker-compose.yml --env-file .env down

build:
	docker exec $(ROS2_CONTAINER) bash -c "\
		source /opt/ros/humble/setup.bash && \
		( [ -f $(ROS2_WS)/install/setup.bash ] && source $(ROS2_WS)/install/setup.bash || true ) && \
		cd $(ROS2_WS) && \
		colcon build $(COLCON_SELECT)"

test:
	# --executor sequential: 複数パッケージを並列実行すると、別パッケージのgtest同士が
	# 同じROSトピック名(例: /safety/anomaly_event)で混信することがあるため直列実行する
	docker exec $(ROS2_CONTAINER) bash -c "\
		source /opt/ros/humble/setup.bash && \
		source $(ROS2_WS)/install/setup.bash && \
		cd $(ROS2_WS) && \
		colcon test --executor sequential $(COLCON_SELECT) && \
		colcon test-result --verbose"

build-test: build test
