#ifndef SAFETY_STATE_MACHINE__SAFETY_STATE_MACHINE_NODE_HPP_
#define SAFETY_STATE_MACHINE__SAFETY_STATE_MACHINE_NODE_HPP_

#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "safety_metrics/prometheus_exporter.hpp"
#include "safety_msgs/msg/anomaly_event.hpp"
#include "std_msgs/msg/bool.hpp"
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
  bool sensors_healthy() const {return sensors_healthy_;}
  bool estop_latched() const {return estop_latched_;}

private:
  void on_anomaly_event(const safety_msgs::msg::AnomalyEvent::SharedPtr msg);
  void on_recovery_command(const std_msgs::msg::Empty::SharedPtr msg);
  void on_nav2_cmd_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void on_sensors_ok(const std_msgs::msg::Bool::SharedPtr msg);
  void on_cmd_vel_timer();
  void on_degraded_timeout();

  void transition_safety_state(SafetyState new_state);

  // パラメータ
  int64_t degraded_timeout_ms_;
  double degraded_speed_scale_;
  int64_t cmd_vel_period_ms_;
  int64_t metrics_port_;

  SafetyState safety_state_;
  // Nav2 Jazzyはvelocity_smoother/controller_serverのenable_stamped_cmd_velが
  // デフォルトtrueでTwistStampedを出すため、中継元もTwistStampedで受ける
  geometry_msgs::msg::Twist latest_nav2_cmd_vel_;
  // watchdogの/safety/sensors_okをそのまま反映(センサー由来のSAFE_STOP中でも、
  // センサー自体が生きていればcmd_vel_nav2を中継してよいかの判定に使う)。
  // watchdogからの通知が届くまでは安全側に倒してfalse(未健全)扱いとする。
  bool sensors_healthy_;
  // estop_bridge由来のCRITICAL(リモートE-Stop)でSAFE_STOPに入った場合は、
  // センサーが健全でもcmd_velを中継しない(E-Stopの意味を上書きしないため)。
  // NORMALへ完全復旧するまで解除しない。
  bool estop_latched_;

  rclcpp::Subscription<safety_msgs::msg::AnomalyEvent>::SharedPtr anomaly_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr recovery_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr nav2_cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sensors_ok_sub_;

  // ros_gz_bridgeのcmd_velはgz.msgs.Twist(ステートレス)とのブリッジにTwistStampedを
  // 要求するため、最終出力のみTwistStamped(内部の中継元latest_nav2_cmd_vel_はTwistのまま)
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>> cmd_vel_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> state_pub_;

  rclcpp::TimerBase::SharedPtr cmd_vel_timer_;
  rclcpp::TimerBase::SharedPtr degraded_timer_;

  safety_metrics::PrometheusExporter metrics_;
};

}  // namespace safety_state_machine

#endif  // SAFETY_STATE_MACHINE__SAFETY_STATE_MACHINE_NODE_HPP_
