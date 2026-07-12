#ifndef SAFETY_STATE_MACHINE__SAFETY_STATE_MACHINE_NODE_HPP_
#define SAFETY_STATE_MACHINE__SAFETY_STATE_MACHINE_NODE_HPP_

#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "safety_metrics/prometheus_exporter.hpp"
#include "safety_msgs/msg/anomaly_event.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

namespace safety_state_machine
{

// hacobotの安全停止ステートマシン本体。
//
// 注意: これはROS2のlifecycle状態(unconfigured/inactive/active/finalized)とは別物。
// lifecycle状態は「このノード自体が起動しているか」を表し、on_activate()中でのみ
// このSafetyState(NORMAL/DEGRADED/SAFE_STOP/MANUAL_RECOVERY)の内部FSMが実際に動作する。
enum class SafetyState
{
  NORMAL,
  DEGRADED,
  SAFE_STOP,
  MANUAL_RECOVERY,
};

std::string to_string(SafetyState state);

class SafetyStateMachineNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit SafetyStateMachineNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  // テスト・デバッグ用に現在のSafetyStateを直接参照できるようにする
  SafetyState current_state() const {return safety_state_;}

private:
  void on_anomaly_event(const safety_msgs::msg::AnomalyEvent::SharedPtr msg);
  void on_recovery_command(const std_msgs::msg::Empty::SharedPtr msg);
  void on_nav2_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_cmd_vel_timer();
  void on_degraded_timeout();

  void transition_safety_state(SafetyState new_state);

  // パラメータ
  int64_t degraded_timeout_ms_;
  double degraded_speed_scale_;
  int64_t cmd_vel_period_ms_;
  int64_t metrics_port_;

  SafetyState safety_state_;
  geometry_msgs::msg::Twist latest_nav2_cmd_vel_;

  rclcpp::Subscription<safety_msgs::msg::AnomalyEvent>::SharedPtr anomaly_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr recovery_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav2_cmd_vel_sub_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>> cmd_vel_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> state_pub_;

  rclcpp::TimerBase::SharedPtr cmd_vel_timer_;
  rclcpp::TimerBase::SharedPtr degraded_timer_;

  safety_metrics::PrometheusExporter metrics_;
};

}  // namespace safety_state_machine

#endif  // SAFETY_STATE_MACHINE__SAFETY_STATE_MACHINE_NODE_HPP_
