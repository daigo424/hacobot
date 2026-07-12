#include "watchdog/watchdog_node.hpp"

#include <chrono>
#include <sstream>

using namespace std::chrono_literals;

namespace watchdog
{

WatchdogNode::WatchdogNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("watchdog", options),
  timeout_ms_(500),
  check_period_ms_(50),
  startup_grace_period_ms_(8000),
  metrics_port_(9103)
{
  // "トピック名@メッセージ型" 形式。GenericSubscriptionの生成に型名が必須なため。
  this->declare_parameter<std::vector<std::string>>(
    "monitored_topics",
    std::vector<std::string>{
    "/scan@sensor_msgs/msg/LaserScan",
    "/camera/image_raw@sensor_msgs/msg/Image",
    "/imu@sensor_msgs/msg/Imu",
  });
  this->declare_parameter<int64_t>("timeout_ms", 500);
  this->declare_parameter<int64_t>("check_period_ms", 50);
  // Gazebo/センサープラグインの起動に数秒かかるため、heartbeat_monitorより長めに取る
  this->declare_parameter<int64_t>("startup_grace_period_ms", 8000);
  this->declare_parameter<int64_t>("metrics_port", 9103);
}

std::vector<WatchdogNode::MonitoredTopic> WatchdogNode::parse_monitored_topics(
  const std::vector<std::string> & raw_entries)
{
  std::vector<MonitoredTopic> result;
  for (const auto & entry : raw_entries) {
    const auto pos = entry.find('@');
    if (pos == std::string::npos) {
      continue;
    }
    MonitoredTopic topic;
    topic.name = entry.substr(0, pos);
    topic.type = entry.substr(pos + 1);
    result.push_back(topic);
  }
  return result;
}

WatchdogNode::CallbackReturn WatchdogNode::on_configure(
  const rclcpp_lifecycle::State & /*state*/)
{
  const auto raw_entries = this->get_parameter("monitored_topics").as_string_array();
  monitored_topics_ = parse_monitored_topics(raw_entries);
  timeout_ms_ = this->get_parameter("timeout_ms").as_int();
  check_period_ms_ = this->get_parameter("check_period_ms").as_int();
  startup_grace_period_ms_ = this->get_parameter("startup_grace_period_ms").as_int();
  metrics_port_ = this->get_parameter("metrics_port").as_int();
  metrics_.start(static_cast<int>(metrics_port_));

  const auto now = this->now();
  last_received_.clear();
  is_stale_.clear();
  for (const auto & topic : monitored_topics_) {
    // configure直後にいきなり異常判定しないよう、猶予として現在時刻で初期化する
    last_received_[topic.name] = now;
    is_stale_[topic.name] = false;
  }

  anomaly_pub_ = this->create_publisher<safety_msgs::msg::AnomalyEvent>(
    "/safety/anomaly_event", rclcpp::QoS(10));

  RCLCPP_INFO(
    this->get_logger(),
    "Configured with %zu monitored topic(s), timeout_ms=%ld, check_period_ms=%ld",
    monitored_topics_.size(), timeout_ms_, check_period_ms_);

  return CallbackReturn::SUCCESS;
}

WatchdogNode::CallbackReturn WatchdogNode::on_activate(
  const rclcpp_lifecycle::State & /*state*/)
{
  anomaly_pub_->on_activate();
  activated_at_ = this->now();

  subs_.clear();
  for (const auto & topic : monitored_topics_) {
    const std::string topic_name = topic.name;
    auto sub = this->create_generic_subscription(
      topic.name, topic.type, rclcpp::SensorDataQoS(),
      [this, topic_name](std::shared_ptr<rclcpp::SerializedMessage> /*msg*/) {
        last_received_[topic_name] = this->now();
        is_stale_[topic_name] = false;
      });
    subs_.push_back(sub);
  }

  recovery_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "/safety/recovery_command", rclcpp::QoS(10),
    std::bind(&WatchdogNode::on_recovery_command, this, std::placeholders::_1));

  check_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(check_period_ms_),
    std::bind(&WatchdogNode::check_timeouts, this));

  RCLCPP_INFO(this->get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

WatchdogNode::CallbackReturn WatchdogNode::on_deactivate(
  const rclcpp_lifecycle::State & /*state*/)
{
  check_timer_.reset();
  subs_.clear();
  recovery_sub_.reset();
  anomaly_pub_->on_deactivate();

  RCLCPP_INFO(this->get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

WatchdogNode::CallbackReturn WatchdogNode::on_cleanup(
  const rclcpp_lifecycle::State & /*state*/)
{
  anomaly_pub_.reset();
  last_received_.clear();
  is_stale_.clear();
  monitored_topics_.clear();
  metrics_.stop();

  RCLCPP_INFO(this->get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

WatchdogNode::CallbackReturn WatchdogNode::on_shutdown(
  const rclcpp_lifecycle::State & /*state*/)
{
  check_timer_.reset();
  subs_.clear();
  recovery_sub_.reset();
  anomaly_pub_.reset();
  metrics_.stop();

  RCLCPP_INFO(this->get_logger(), "Shutdown");
  return CallbackReturn::SUCCESS;
}

void WatchdogNode::on_recovery_command(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  // safety_state_machineがNORMALへ復旧しても、監視対象が実際には復旧していない場合、
  // is_stale_をクリアして次のcheck_timeouts()で再度異常を検知・通知できるようにする。
  for (auto & entry : is_stale_) {
    entry.second = false;
  }
  RCLCPP_INFO(
    this->get_logger(),
    "Received recovery_command: cleared stale flags for %zu monitored topic(s)",
    is_stale_.size());
}

void WatchdogNode::check_timeouts()
{
  const auto now = this->now();
  const auto since_activation_ms = (now - activated_at_).nanoseconds() / 1000000;
  const auto effective_timeout_ms =
    (since_activation_ms < startup_grace_period_ms_) ? startup_grace_period_ms_ : timeout_ms_;

  for (const auto & topic : monitored_topics_) {
    const auto & name = topic.name;
    const auto elapsed_ms = (now - last_received_[name]).nanoseconds() / 1000000;
    metrics_.set_gauge(
      "watchdog_topic_stale{topic=\"" + name + "\"}",
      is_stale_[name] ? 1.0 : 0.0,
      "1 if this monitored sensor topic is currently considered stale");

    if (elapsed_ms >= effective_timeout_ms && !is_stale_[name]) {
      is_stale_[name] = true;

      safety_msgs::msg::AnomalyEvent event;
      event.stamp = now;
      event.source = name;
      // センサー途絶は安全上即座に完全停止すべきためCRITICAL(DEGRADEDを経由しない)
      event.severity = safety_msgs::msg::AnomalyEvent::CRITICAL;
      std::ostringstream reason;
      reason << "sensor topic timeout: " << elapsed_ms <<
        "ms since last message on " << name << " (threshold " << effective_timeout_ms << "ms)";
      event.reason = reason.str();

      if (anomaly_pub_->is_activated()) {
        anomaly_pub_->publish(event);
      }
      metrics_.increment_counter(
        "watchdog_anomaly_total{topic=\"" + name + "\"}",
        "Number of sensor topic timeout anomalies detected, by topic");
      RCLCPP_ERROR(this->get_logger(), "Anomaly detected: %s", event.reason.c_str());
    }
  }
}

}  // namespace watchdog
