#include "heartbeat_monitor/heartbeat_monitor_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace heartbeat_monitor
{

HeartbeatMonitorNode::HeartbeatMonitorNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("heartbeat_monitor", options),
  liveliness_lease_ms_(300),
  // 起動直後のCPU競合で誤検知が起きたため30000ms->40000msに拡大
  // (詳細・実測根拠はREADME「既知の制約」参照)
  startup_grace_period_ms_(40000),
  metrics_port_(9101)
{
  // センサー系の生存監視はwatchdogが実トピックを直接監視する設計のため、
  // heartbeat_monitorの監視対象には含めない
  this->declare_parameter<std::vector<std::string>>(
    "monitored_sources",
    std::vector<std::string>{"nav2", "comm_bridge"});
  // publisher側(estop_bridge/nav2_heartbeat_adapter)のheartbeat_lease_msと
  // 一致させること(DDS Liveliness QoSのlease durationとして両者が対称である必要がある)。
  this->declare_parameter<int64_t>("liveliness_lease_ms", 300);
  // nav2_heartbeat_adapter自身の猶予で通常はカバーされる想定だが、二重の保険として同値に揃える
  this->declare_parameter<int64_t>("startup_grace_period_ms", 40000);
  this->declare_parameter<int64_t>("metrics_port", 9101);
}

HeartbeatMonitorNode::CallbackReturn HeartbeatMonitorNode::on_configure(
  const rclcpp_lifecycle::State & /*state*/)
{
  monitored_sources_ = this->get_parameter("monitored_sources").as_string_array();
  liveliness_lease_ms_ = this->get_parameter("liveliness_lease_ms").as_int();
  startup_grace_period_ms_ = this->get_parameter("startup_grace_period_ms").as_int();
  metrics_port_ = this->get_parameter("metrics_port").as_int();
  metrics_.start(static_cast<int>(metrics_port_));

  currently_alive_.clear();
  ever_alive_.clear();
  is_stale_.clear();
  for (const auto & source : monitored_sources_) {
    // configure直後にいきなり異常判定しないよう、生存している前提で楽観初期化する
    // (DDSマッチング未完了時の非生存報告はstartup_grace_period_ms内では無視される)。
    currently_alive_[source] = true;
    ever_alive_[source] = false;
    is_stale_[source] = false;
  }

  anomaly_pub_ = this->create_publisher<safety_msgs::msg::AnomalyEvent>(
    "/safety/anomaly_event", rclcpp::QoS(10));

  RCLCPP_INFO(
    this->get_logger(),
    "Configured with %zu monitored source(s), liveliness_lease_ms=%ld",
    monitored_sources_.size(), liveliness_lease_ms_);

  return CallbackReturn::SUCCESS;
}

HeartbeatMonitorNode::CallbackReturn HeartbeatMonitorNode::on_activate(
  const rclcpp_lifecycle::State & /*state*/)
{
  anomaly_pub_->on_activate();
  activated_at_ = this->now();

  heartbeat_subs_.clear();
  const rclcpp::QoS heartbeat_qos = rclcpp::QoS(10)
    .liveliness(rclcpp::LivelinessPolicy::ManualByTopic)
    .liveliness_lease_duration(std::chrono::milliseconds(liveliness_lease_ms_));

  for (const auto & source : monitored_sources_) {
    const std::string topic = "/safety/heartbeat/" + source;
    rclcpp::SubscriptionOptions options;
    options.event_callbacks.liveliness_callback =
      [this, source](rclcpp::QOSLivelinessChangedInfo & info) {
        on_liveliness_changed(source, info);
      };
    auto sub = this->create_subscription<std_msgs::msg::Empty>(
      topic, heartbeat_qos,
      [](const std_msgs::msg::Empty::SharedPtr /*msg*/) {
        // 生存判定はliveliness_callback側で行うため、メッセージ本体は使わない。
      },
      options);
    heartbeat_subs_.push_back(sub);
  }

  recovery_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "/safety/recovery_command", rclcpp::QoS(10),
    std::bind(&HeartbeatMonitorNode::on_recovery_command, this, std::placeholders::_1));

  // liveliness_callbackはalive/not_aliveの「変化」でしか発火せず、一度も生存を
  // 主張しないpublisher(起動失敗等)を検知できないため、起動猶予終了時に一度だけ確認する。
  startup_check_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(startup_grace_period_ms_),
    std::bind(&HeartbeatMonitorNode::check_startup_liveliness, this));

  RCLCPP_INFO(this->get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

HeartbeatMonitorNode::CallbackReturn HeartbeatMonitorNode::on_deactivate(
  const rclcpp_lifecycle::State & /*state*/)
{
  startup_check_timer_.reset();
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
  currently_alive_.clear();
  ever_alive_.clear();
  is_stale_.clear();
  monitored_sources_.clear();
  metrics_.stop();

  RCLCPP_INFO(this->get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

HeartbeatMonitorNode::CallbackReturn HeartbeatMonitorNode::on_shutdown(
  const rclcpp_lifecycle::State & /*state*/)
{
  startup_check_timer_.reset();
  heartbeat_subs_.clear();
  recovery_sub_.reset();
  anomaly_pub_.reset();
  metrics_.stop();

  RCLCPP_INFO(this->get_logger(), "Shutdown");
  return CallbackReturn::SUCCESS;
}

bool HeartbeatMonitorNode::in_startup_grace() const
{
  const auto since_activation_ms = (this->now() - activated_at_).nanoseconds() / 1000000;
  return since_activation_ms < startup_grace_period_ms_;
}

void HeartbeatMonitorNode::on_liveliness_changed(
  const std::string & source, rclcpp::QOSLivelinessChangedInfo & info)
{
  const bool alive = info.alive_count > 0;
  const bool was_alive = currently_alive_[source];
  currently_alive_[source] = alive;
  if (alive) {
    ever_alive_[source] = true;
  }

  metrics_.set_gauge(
    "heartbeat_monitor_source_stale{source=\"" + source + "\"}",
    is_stale_[source] ? 1.0 : 0.0,
    "1 if this monitored source's heartbeat is currently considered stale");

  if (!alive && was_alive && !in_startup_grace()) {
    publish_anomaly(source);
  }
}

void HeartbeatMonitorNode::on_recovery_command(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  // is_stale_クリア直後にcurrently_alive_を見て即座に再検知する(ポーリングタイマーが
  // 無いため、新しいliveliness_callback発火を待たずここで明示的にチェックする)。
  for (auto & entry : is_stale_) {
    entry.second = false;
  }
  RCLCPP_INFO(
    this->get_logger(),
    "Received recovery_command: cleared stale flags for %zu monitored source(s)",
    is_stale_.size());

  for (const auto & source : monitored_sources_) {
    if (!currently_alive_[source]) {
      publish_anomaly(source);
    }
  }
}

void HeartbeatMonitorNode::check_startup_liveliness()
{
  startup_check_timer_->cancel();
  for (const auto & source : monitored_sources_) {
    // ever_alive_==falseは一度も生存を主張していないケース、currently_alive_==falseは
    // 猶予中に非生存へ遷移し抑制されたケース。どちらも猶予終了後は改めて異常として扱う。
    if (!ever_alive_[source] || !currently_alive_[source]) {
      currently_alive_[source] = false;
      publish_anomaly(source);
    }
  }
}

void HeartbeatMonitorNode::publish_anomaly(const std::string & source)
{
  if (is_stale_[source]) {
    return;
  }
  is_stale_[source] = true;
  metrics_.set_gauge(
    "heartbeat_monitor_source_stale{source=\"" + source + "\"}", 1.0,
    "1 if this monitored source's heartbeat is currently considered stale");

  safety_msgs::msg::AnomalyEvent event;
  event.stamp = this->now();
  event.source = source;
  // heartbeat途絶は設計方針上WARNING(DEGRADED経由)として扱う。
  // センサー途絶(CRITICAL、即SAFE_STOP)はwatchdogが担当する。
  event.severity = safety_msgs::msg::AnomalyEvent::WARNING;
  event.reason = "heartbeat liveliness lost (lease " +
    std::to_string(liveliness_lease_ms_) + "ms)";

  if (anomaly_pub_->is_activated()) {
    anomaly_pub_->publish(event);
  }
  metrics_.increment_counter(
    "heartbeat_monitor_anomaly_total{source=\"" + source + "\"}",
    "Number of heartbeat timeout anomalies detected, by monitored source");
  RCLCPP_WARN(this->get_logger(), "Anomaly detected: %s", event.reason.c_str());
}

}  // namespace heartbeat_monitor
