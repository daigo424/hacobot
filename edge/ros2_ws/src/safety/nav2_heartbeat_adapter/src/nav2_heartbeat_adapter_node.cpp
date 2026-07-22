#include "nav2_heartbeat_adapter/nav2_heartbeat_adapter_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace nav2_heartbeat_adapter
{

Nav2HeartbeatAdapterNode::Nav2HeartbeatAdapterNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("nav2_heartbeat_adapter", options),
  liveness_topic_("/local_costmap/costmap"),
  // このHumbleビルドのnav2_costmap_2dは/local_costmap/costmapに
  // nav_msgs/msg/OccupancyGridとnav2_msgs/msg/Costmapの2つの型が
  // 同じトピック名で"存在"しうるが、実測では後者は一度もpublishされない
  // (40秒待っても0件)。前者(OccupancyGrid)は起動直後から確実にpublishされる
  // ことを確認済みのため、liveness監視にはこちらを使う。
  liveness_topic_type_("nav_msgs/msg/OccupancyGrid"),
  // local_costmapの公称publish間隔(publish_frequency=2Hz=500ms)に対し、
  // このhostはk3s/Kafka/VSCode等の並行負荷が常時あり定常状態でも500ms周期の
  // ジッターが大きい可能性があるため、余裕を持たせた値にする。
  liveness_window_ms_(3000),
  heartbeat_period_ms_(100),
  // 起動直後のCPU競合で誤検知が起きたため30000ms->40000msに拡大
  // (詳細・実測根拠はREADME「既知の制約」参照)。この間はliveness_topicの受信有無に
  // 関わらずハートビートをpublishし続け、起動中を異常と誤検知しないようにする。
  startup_grace_period_ms_(40000),
  metrics_port_(9104)
{
  this->declare_parameter<std::string>("liveness_topic", liveness_topic_);
  this->declare_parameter<std::string>("liveness_topic_type", liveness_topic_type_);
  this->declare_parameter<int64_t>("liveness_window_ms", liveness_window_ms_);
  this->declare_parameter<int64_t>("heartbeat_period_ms", heartbeat_period_ms_);
  this->declare_parameter<int64_t>("startup_grace_period_ms", startup_grace_period_ms_);
  this->declare_parameter<int64_t>("metrics_port", metrics_port_);
}

Nav2HeartbeatAdapterNode::CallbackReturn Nav2HeartbeatAdapterNode::on_configure(
  const rclcpp_lifecycle::State & /*state*/)
{
  liveness_topic_ = this->get_parameter("liveness_topic").as_string();
  liveness_topic_type_ = this->get_parameter("liveness_topic_type").as_string();
  liveness_window_ms_ = this->get_parameter("liveness_window_ms").as_int();
  heartbeat_period_ms_ = this->get_parameter("heartbeat_period_ms").as_int();
  startup_grace_period_ms_ = this->get_parameter("startup_grace_period_ms").as_int();
  metrics_port_ = this->get_parameter("metrics_port").as_int();
  metrics_.start(static_cast<int>(metrics_port_));

  // configure直後にいきなり「死んでいる」と判定しないよう、猶予として現在時刻で初期化する
  last_seen_ = this->now();

  heartbeat_pub_ = this->create_publisher<std_msgs::msg::Empty>(
    "/safety/heartbeat/nav2", rclcpp::QoS(10));

  RCLCPP_INFO(
    this->get_logger(),
    "Configured (liveness_topic=%s, liveness_window_ms=%ld, heartbeat_period_ms=%ld, "
    "startup_grace_period_ms=%ld)",
    liveness_topic_.c_str(), liveness_window_ms_, heartbeat_period_ms_,
    startup_grace_period_ms_);

  return CallbackReturn::SUCCESS;
}

Nav2HeartbeatAdapterNode::CallbackReturn Nav2HeartbeatAdapterNode::on_activate(
  const rclcpp_lifecycle::State & /*state*/)
{
  heartbeat_pub_->on_activate();
  activated_at_ = this->now();

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
  const auto now = this->now();
  const auto since_activation_ms = (now - activated_at_).nanoseconds() / 1000000;
  const auto elapsed_ms = (now - last_seen_).nanoseconds() / 1000000;
  const bool in_startup_grace = since_activation_ms < startup_grace_period_ms_;
  const bool alive = in_startup_grace || (elapsed_ms <= liveness_window_ms_);
  metrics_.set_gauge(
    "nav2_heartbeat_adapter_nav2_alive", alive ? 1.0 : 0.0,
    "1 if Nav2's liveness topic has been seen within liveness_window_ms "
    "(or still within startup_grace_period_ms)");

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
