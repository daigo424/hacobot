#include "heartbeat_monitor/heartbeat_monitor_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace heartbeat_monitor
{

HeartbeatMonitorNode::HeartbeatMonitorNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("heartbeat_monitor", options),
  timeout_ms_(300),
  check_period_ms_(50),
  startup_grace_period_ms_(5000),
  metrics_port_(9101)
{
  // センサー系の生存監視はwatchdogが実トピックを直接監視する設計のため、
  // heartbeat_monitorの監視対象には含めない
  this->declare_parameter<std::vector<std::string>>(
    "monitored_sources",
    std::vector<std::string>{"nav2", "comm_bridge"});
  this->declare_parameter<int64_t>("timeout_ms", 300);
  this->declare_parameter<int64_t>("check_period_ms", 50);
  // Nav2フルスタックの起動に数秒かかる(実機統合テストで実測: 3秒強)ため、
  // それより十分大きい値をデフォルトにする
  this->declare_parameter<int64_t>("startup_grace_period_ms", 5000);
  this->declare_parameter<int64_t>("metrics_port", 9101);
}

HeartbeatMonitorNode::CallbackReturn HeartbeatMonitorNode::on_configure(
  const rclcpp_lifecycle::State & /*state*/)
{
  monitored_sources_ = this->get_parameter("monitored_sources").as_string_array();
  timeout_ms_ = this->get_parameter("timeout_ms").as_int();
  check_period_ms_ = this->get_parameter("check_period_ms").as_int();
  startup_grace_period_ms_ = this->get_parameter("startup_grace_period_ms").as_int();
  metrics_port_ = this->get_parameter("metrics_port").as_int();
  metrics_.start(static_cast<int>(metrics_port_));

  const auto now = this->now();
  last_heartbeat_.clear();
  is_stale_.clear();
  for (const auto & source : monitored_sources_) {
    // configure直後にいきなり異常判定しないよう、猶予として現在時刻で初期化する
    last_heartbeat_[source] = now;
    is_stale_[source] = false;
  }

  anomaly_pub_ = this->create_publisher<safety_msgs::msg::AnomalyEvent>(
    "/safety/anomaly_event", rclcpp::QoS(10));

  RCLCPP_INFO(
    this->get_logger(),
    "Configured with %zu monitored source(s), timeout_ms=%ld, check_period_ms=%ld",
    monitored_sources_.size(), timeout_ms_, check_period_ms_);

  return CallbackReturn::SUCCESS;
}

HeartbeatMonitorNode::CallbackReturn HeartbeatMonitorNode::on_activate(
  const rclcpp_lifecycle::State & /*state*/)
{
  anomaly_pub_->on_activate();
  activated_at_ = this->now();

  heartbeat_subs_.clear();
  for (const auto & source : monitored_sources_) {
    const std::string topic = "/safety/heartbeat/" + source;
    auto sub = this->create_subscription<std_msgs::msg::Empty>(
      topic, rclcpp::QoS(10),
      [this, source](const std_msgs::msg::Empty::SharedPtr /*msg*/) {
        last_heartbeat_[source] = this->now();
        is_stale_[source] = false;
      });
    heartbeat_subs_.push_back(sub);
  }

  recovery_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "/safety/recovery_command", rclcpp::QoS(10),
    std::bind(&HeartbeatMonitorNode::on_recovery_command, this, std::placeholders::_1));

  check_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(check_period_ms_),
    std::bind(&HeartbeatMonitorNode::check_timeouts, this));

  RCLCPP_INFO(this->get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

HeartbeatMonitorNode::CallbackReturn HeartbeatMonitorNode::on_deactivate(
  const rclcpp_lifecycle::State & /*state*/)
{
  check_timer_.reset();
  heartbeat_subs_.clear();
  recovery_sub_.reset();
  anomaly_pub_->on_deactivate();

  RCLCPP_INFO(this->get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

HeartbeatMonitorNode::CallbackReturn HeartbeatMonitorNode::on_cleanup(
  const rclcpp_lifecycle::State & /*state*/)
{
  anomaly_pub_.reset();
  last_heartbeat_.clear();
  is_stale_.clear();
  monitored_sources_.clear();
  metrics_.stop();

  RCLCPP_INFO(this->get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

HeartbeatMonitorNode::CallbackReturn HeartbeatMonitorNode::on_shutdown(
  const rclcpp_lifecycle::State & /*state*/)
{
  check_timer_.reset();
  heartbeat_subs_.clear();
  recovery_sub_.reset();
  anomaly_pub_.reset();
  metrics_.stop();

  RCLCPP_INFO(this->get_logger(), "Shutdown");
  return CallbackReturn::SUCCESS;
}

void HeartbeatMonitorNode::on_recovery_command(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  // safety_state_machineがNORMALへ復旧しても、監視対象が実際には復旧していない場合、
  // is_stale_をクリアして次のcheck_timeouts()で再度異常を検知・通知できるようにする。
  for (auto & entry : is_stale_) {
    entry.second = false;
  }
  RCLCPP_INFO(
    this->get_logger(),
    "Received recovery_command: cleared stale flags for %zu monitored source(s)",
    is_stale_.size());
}

void HeartbeatMonitorNode::check_timeouts()
{
  const auto now = this->now();
  const auto since_activation_ms = (now - activated_at_).nanoseconds() / 1000000;
  const auto effective_timeout_ms =
    (since_activation_ms < startup_grace_period_ms_) ? startup_grace_period_ms_ : timeout_ms_;

  for (const auto & source : monitored_sources_) {
    const auto elapsed_ms = (now - last_heartbeat_[source]).nanoseconds() / 1000000;
    metrics_.set_gauge(
      "heartbeat_monitor_source_stale{source=\"" + source + "\"}",
      is_stale_[source] ? 1.0 : 0.0,
      "1 if this monitored source's heartbeat is currently considered stale");

    if (elapsed_ms >= effective_timeout_ms && !is_stale_[source]) {
      is_stale_[source] = true;

      safety_msgs::msg::AnomalyEvent event;
      event.stamp = now;
      event.source = source;
      // heartbeat途絶は設計方針上WARNING(DEGRADED経由)として扱う。
      // センサー途絶(CRITICAL、即SAFE_STOP)はwatchdogが担当する。
      event.severity = safety_msgs::msg::AnomalyEvent::WARNING;
      event.reason = "heartbeat timeout: " + std::to_string(elapsed_ms) +
        "ms since last heartbeat (threshold " + std::to_string(effective_timeout_ms) + "ms)";

      if (anomaly_pub_->is_activated()) {
        anomaly_pub_->publish(event);
      }
      metrics_.increment_counter(
        "heartbeat_monitor_anomaly_total{source=\"" + source + "\"}",
        "Number of heartbeat timeout anomalies detected, by monitored source");
      RCLCPP_WARN(this->get_logger(), "Anomaly detected: %s", event.reason.c_str());
    }
  }
}

}  // namespace heartbeat_monitor
