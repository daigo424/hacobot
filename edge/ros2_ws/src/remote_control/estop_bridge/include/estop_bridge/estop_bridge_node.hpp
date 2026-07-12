#ifndef ESTOP_BRIDGE__ESTOP_BRIDGE_NODE_HPP_
#define ESTOP_BRIDGE__ESTOP_BRIDGE_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "librdkafka/rdkafkacpp.h"
#include "safety_metrics/prometheus_exporter.hpp"
#include "safety_msgs/msg/anomaly_event.hpp"
#include "std_msgs/msg/empty.hpp"

namespace estop_bridge
{

// Kafkaの robot-control トピックを購読し、リモートE-Stopコマンドを受信したら
// /safety/anomaly_event へ severity="CRITICAL" のAnomalyEventを最優先でpublishするノード。
//
// 設計上の要点:
// - Kafkaコンシューマは専用のstd::threadで動かす。ROS2の実行器(executor)が
//   Nav2やセンサー系の重いコールバックで詰まっていても、E-Stop検知はそれに影響されず
//   低遅延で処理できるようにするため(「他のセンサー処理系とは独立したスレッド/
//   コールバックグループ」という要件をstd::threadの分離で満たす)。
// - クラウド接続の生死監視(TCP到達性チェックによるハートビート)もこのノードが担う。
//   E-Stop受信スレッドとは別に、heartbeat_timer_/connectivity_check_timer_は
//   通常のROSタイマー(executor側)で動かす -- ハートビート発行の遅延はE-Stopほど
//   致命的ではないため、あえてスレッドを分ける必要はない。
class EstopBridgeNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit EstopBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~EstopBridgeNode() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  // Kafkaメッセージのペイロードがリモート E-Stop コマンドを表すかどうかを判定する。
  // Kafka I/Oに依存しない純粋関数として切り出し、gtestで単体テストできるようにしている。
  static bool is_estop_command(const std::string & payload);

private:
  void kafka_consume_loop();
  void on_heartbeat_timer();
  void on_connectivity_check_timer();
  static bool check_tcp_reachable(const std::string & host, int port, int timeout_ms);
  void stop_consumer_thread();

  // --- Kafka購読(E-Stop本体)関連パラメータ ---
  std::string kafka_brokers_;
  std::string kafka_topic_;
  std::string kafka_group_id_;
  int64_t kafka_poll_timeout_ms_;

  // --- 通信ブリッジ生存確認(旧comm_bridge_heartbeatを統合) ---
  std::string kafka_host_;
  int64_t kafka_port_;
  int64_t connect_timeout_ms_;
  int64_t connectivity_check_period_ms_;
  int64_t heartbeat_period_ms_;
  bool assume_healthy_;
  std::atomic<bool> is_healthy_;
  int64_t metrics_port_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<safety_msgs::msg::AnomalyEvent>>
    anomaly_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Empty>> heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr connectivity_check_timer_;

  std::unique_ptr<RdKafka::KafkaConsumer> consumer_;
  std::thread consumer_thread_;
  std::atomic<bool> consumer_running_;

  safety_metrics::PrometheusExporter metrics_;
};

}  // namespace estop_bridge

#endif  // ESTOP_BRIDGE__ESTOP_BRIDGE_NODE_HPP_
