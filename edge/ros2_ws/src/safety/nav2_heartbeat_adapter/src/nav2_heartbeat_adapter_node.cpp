#include "nav2_heartbeat_adapter/nav2_heartbeat_adapter_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace nav2_heartbeat_adapter
{

Nav2HeartbeatAdapterNode::Nav2HeartbeatAdapterNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("nav2_heartbeat_adapter", options),
  liveness_topic_("/local_costmap/costmap"),
  liveness_topic_type_("nav2_msgs/msg/Costmap"),
  liveness_window_ms_(500),
  heartbeat_period_ms_(100),
  metrics_port_(9104)
{
  this->declare_parameter<std::string>("liveness_topic", liveness_topic_);
  this->declare_parameter<std::string>("liveness_topic_type", liveness_topic_type_);
  this->declare_parameter<int64_t>("liveness_window_ms", liveness_window_ms_);
  this->declare_parameter<int64_t>("heartbeat_period_ms", heartbeat_period_ms_);
  this->declare_parameter<int64_t>("metrics_port", metrics_port_);
}

Nav2HeartbeatAdapterNode::CallbackReturn Nav2HeartbeatAdapterNode::on_configure(
  const rclcpp_lifecycle::State & /*state*/)
{
  liveness_topic_ = this->get_parameter("liveness_topic").as_string();
  liveness_topic_type_ = this->get_parameter("liveness_topic_type").as_string();
  liveness_window_ms_ = this->get_parameter("liveness_window_ms").as_int();
  heartbeat_period_ms_ = this->get_parameter("heartbeat_period_ms").as_int();
  metrics_port_ = this->get_parameter("metrics_port").as_int();
  metrics_.start(static_cast<int>(metrics_port_));

  // configure直後にいきなり「死んでいる」と判定しないよう、猶予として現在時刻で初期化する
  last_seen_ = this->now();

  heartbeat_pub_ = this->create_publisher<std_msgs::msg::Empty>(
    "/safety/heartbeat/nav2", rclcpp::QoS(10));

  RCLCPP_INFO(
    this->get_logger(),
    "Configured (liveness_topic=%s, liveness_window_ms=%ld, heartbeat_period_ms=%ld)",
    liveness_topic_.c_str(), liveness_window_ms_, heartbeat_period_ms_);

  return CallbackReturn::SUCCESS;
}

Nav2HeartbeatAdapterNode::CallbackReturn Nav2HeartbeatAdapterNode::on_activate(
  const rclcpp_lifecycle::State & /*state*/)
{
  heartbeat_pub_->on_activate();

  liveness_sub_ = this->create_generic_subscription(
    liveness_topic_, liveness_topic_type_, rclcpp::SensorDataQoS(),
    [this](std::shared_ptr<rclcpp::SerializedMessage> /*msg*/) {
      last_seen_ = this->now();
    });

  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(heartbeat_period_ms_),
    std::bind(&Nav2HeartbeatAdapterNode::on_heartbeat_timer, this));

  RCLCPP_INFO(this->get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

Nav2HeartbeatAdapterNode::CallbackReturn Nav2HeartbeatAdapterNode::on_deactivate(
  const rclcpp_lifecycle::State & /*state*/)
{
  heartbeat_timer_.reset();
  liveness_sub_.reset();
  heartbeat_pub_->on_deactivate();

  RCLCPP_INFO(this->get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

Nav2HeartbeatAdapterNode::CallbackReturn Nav2HeartbeatAdapterNode::on_cleanup(
  const rclcpp_lifecycle::State & /*state*/)
{
  heartbeat_pub_.reset();
  metrics_.stop();

  RCLCPP_INFO(this->get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

Nav2HeartbeatAdapterNode::CallbackReturn Nav2HeartbeatAdapterNode::on_shutdown(
  const rclcpp_lifecycle::State & /*state*/)
{
  heartbeat_timer_.reset();
  liveness_sub_.reset();
  heartbeat_pub_.reset();
  metrics_.stop();

  RCLCPP_INFO(this->get_logger(), "Shutdown");
  return CallbackReturn::SUCCESS;
}

void Nav2HeartbeatAdapterNode::on_heartbeat_timer()
{
  const auto elapsed_ms = (this->now() - last_seen_).nanoseconds() / 1000000;
  const bool alive = elapsed_ms <= liveness_window_ms_;
  metrics_.set_gauge(
    "nav2_heartbeat_adapter_nav2_alive", alive ? 1.0 : 0.0,
    "1 if Nav2's liveness topic has been seen within liveness_window_ms");

  if (!alive) {
    // Nav2の代表トピックが途絶している = Nav2が死んでいるとみなし、
    // ハートビートのpublishを止める(heartbeat_monitor側でタイムアウト検知させる)
    return;
  }

  if (heartbeat_pub_->is_activated()) {
    heartbeat_pub_->publish(std_msgs::msg::Empty());
  }
}

}  // namespace nav2_heartbeat_adapter
