#include "safety_state_machine/safety_state_machine_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace safety_state_machine
{

std::string to_string(SafetyState state)
{
  switch (state) {
    case SafetyState::NORMAL: return "NORMAL";
    case SafetyState::DEGRADED: return "DEGRADED";
    case SafetyState::SAFE_STOP: return "SAFE_STOP";
    case SafetyState::MANUAL_RECOVERY: return "MANUAL_RECOVERY";
  }
  return "UNKNOWN";
}

// Grafanaで時系列パネルとして描画しやすいよう、SafetyStateを数値に対応させる
// (悪化するほど値が大きくなる順序)
double to_metric_value(SafetyState state)
{
  switch (state) {
    case SafetyState::NORMAL: return 0.0;
    case SafetyState::DEGRADED: return 1.0;
    case SafetyState::SAFE_STOP: return 2.0;
    case SafetyState::MANUAL_RECOVERY: return 3.0;
  }
  return -1.0;
}

SafetyStateMachineNode::SafetyStateMachineNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("safety_state_machine", options),
  degraded_timeout_ms_(2000),
  degraded_speed_scale_(0.5),
  cmd_vel_period_ms_(50),
  metrics_port_(9102),
  safety_state_(SafetyState::NORMAL),
  sensors_healthy_(false),
  estop_latched_(false)
{
  this->declare_parameter<int64_t>("degraded_timeout_ms", 2000);
  this->declare_parameter<double>("degraded_speed_scale", 0.5);
  this->declare_parameter<int64_t>("cmd_vel_period_ms", 50);
  this->declare_parameter<int64_t>("metrics_port", 9102);
}

SafetyStateMachineNode::CallbackReturn SafetyStateMachineNode::on_configure(
  const rclcpp_lifecycle::State & /*state*/)
{
  degraded_timeout_ms_ = this->get_parameter("degraded_timeout_ms").as_int();
  degraded_speed_scale_ = this->get_parameter("degraded_speed_scale").as_double();
  cmd_vel_period_ms_ = this->get_parameter("cmd_vel_period_ms").as_int();
  metrics_port_ = this->get_parameter("metrics_port").as_int();
  metrics_.start(static_cast<int>(metrics_port_));

  safety_state_ = SafetyState::NORMAL;
  latest_nav2_cmd_vel_ = geometry_msgs::msg::Twist();

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/cmd_vel", rclcpp::QoS(10));

  // 遅れて起動したノードでも直近の状態がすぐ分かるようtransient_local(latch相当)にする
  state_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/safety/state", rclcpp::QoS(1).transient_local());

  RCLCPP_INFO(
    this->get_logger(),
    "Configured (degraded_timeout_ms=%ld, degraded_speed_scale=%.2f, cmd_vel_period_ms=%ld)",
    degraded_timeout_ms_, degraded_speed_scale_, cmd_vel_period_ms_);

  return CallbackReturn::SUCCESS;
}

SafetyStateMachineNode::CallbackReturn SafetyStateMachineNode::on_activate(
  const rclcpp_lifecycle::State & /*state*/)
{
  cmd_vel_pub_->on_activate();
  state_pub_->on_activate();

  anomaly_sub_ = this->create_subscription<safety_msgs::msg::AnomalyEvent>(
    "/safety/anomaly_event", rclcpp::QoS(10),
    std::bind(&SafetyStateMachineNode::on_anomaly_event, this, std::placeholders::_1));

  recovery_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "/safety/recovery_command", rclcpp::QoS(10),
    std::bind(&SafetyStateMachineNode::on_recovery_command, this, std::placeholders::_1));

  nav2_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/cmd_vel_nav2", rclcpp::QoS(10),
    std::bind(&SafetyStateMachineNode::on_nav2_cmd_vel, this, std::placeholders::_1));

  // watchdogがtransient_localでlatchしているため、watchdogの方が先にactivateしていれば
  // 購読直後に直近の値を受け取れる
  sensors_ok_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/safety/sensors_ok", rclcpp::QoS(1).transient_local(),
    std::bind(&SafetyStateMachineNode::on_sensors_ok, this, std::placeholders::_1));

  cmd_vel_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(cmd_vel_period_ms_),
    std::bind(&SafetyStateMachineNode::on_cmd_vel_timer, this));

  // activate直後は常にNORMALから開始する(意図しない自律再開を防ぐため、
  // 前回のSAFE_STOP状態を暗黙に引き継ぐことはしない)
  safety_state_ = SafetyState::NORMAL;
  // watchdogからの通知を受け取るまでは安全側(未健全)扱いにする
  sensors_healthy_ = false;
  estop_latched_ = false;
  std_msgs::msg::String state_msg;
  state_msg.data = to_string(safety_state_);
  state_pub_->publish(state_msg);
  metrics_.set_gauge(
    "safety_state_machine_state", to_metric_value(safety_state_),
    "Current SafetyState (0=NORMAL, 1=DEGRADED, 2=SAFE_STOP, 3=MANUAL_RECOVERY)");

  RCLCPP_INFO(this->get_logger(), "Activated (state=%s)", to_string(safety_state_).c_str());
  return CallbackReturn::SUCCESS;
}

SafetyStateMachineNode::CallbackReturn SafetyStateMachineNode::on_deactivate(
  const rclcpp_lifecycle::State & /*state*/)
{
  cmd_vel_timer_.reset();
  degraded_timer_.reset();
  anomaly_sub_.reset();
  recovery_sub_.reset();
  nav2_cmd_vel_sub_.reset();
  sensors_ok_sub_.reset();

  cmd_vel_pub_->on_deactivate();
  state_pub_->on_deactivate();

  RCLCPP_INFO(this->get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

SafetyStateMachineNode::CallbackReturn SafetyStateMachineNode::on_cleanup(
  const rclcpp_lifecycle::State & /*state*/)
{
  cmd_vel_pub_.reset();
  state_pub_.reset();
  safety_state_ = SafetyState::NORMAL;
  metrics_.stop();

  RCLCPP_INFO(this->get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

SafetyStateMachineNode::CallbackReturn SafetyStateMachineNode::on_shutdown(
  const rclcpp_lifecycle::State & /*state*/)
{
  cmd_vel_timer_.reset();
  degraded_timer_.reset();
  anomaly_sub_.reset();
  recovery_sub_.reset();
  nav2_cmd_vel_sub_.reset();
  sensors_ok_sub_.reset();
  cmd_vel_pub_.reset();
  state_pub_.reset();
  metrics_.stop();

  RCLCPP_INFO(this->get_logger(), "Shutdown");
  return CallbackReturn::SUCCESS;
}

void SafetyStateMachineNode::on_anomaly_event(
  const safety_msgs::msg::AnomalyEvent::SharedPtr msg)
{
  if (msg->severity == safety_msgs::msg::AnomalyEvent::CRITICAL) {
    // CRITICALは現在の状態に関わらず、DEGRADEDを経由せず即座にSAFE_STOPへ
    RCLCPP_WARN(
      this->get_logger(), "CRITICAL anomaly from '%s': %s -> immediate SAFE_STOP",
      msg->source.c_str(), msg->reason.c_str());
    if (msg->source == "estop_bridge") {
      // リモートE-Stopは、センサーが健全に戻ってもcmd_velを中継してはいけない
      estop_latched_ = true;
    }
    transition_safety_state(SafetyState::SAFE_STOP);
    return;
  }

  if (msg->severity == safety_msgs::msg::AnomalyEvent::WARNING) {
    if (safety_state_ == SafetyState::NORMAL) {
      RCLCPP_WARN(
        this->get_logger(), "WARNING anomaly from '%s': %s -> DEGRADED",
        msg->source.c_str(), msg->reason.c_str());
      transition_safety_state(SafetyState::DEGRADED);
    } else if (safety_state_ == SafetyState::DEGRADED) {
      // DEGRADED中に2件目のWARNINGを受けたら即座にエスカレーション
      RCLCPP_WARN(
        this->get_logger(),
        "Additional WARNING anomaly from '%s' while DEGRADED: %s -> SAFE_STOP",
        msg->source.c_str(), msg->reason.c_str());
      transition_safety_state(SafetyState::SAFE_STOP);
    }
    // SAFE_STOP/MANUAL_RECOVERY中のWARNINGは無視(既に停止している/回復中のため)
  }
}

void SafetyStateMachineNode::on_recovery_command(
  const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  // 2段階の明示的操作を要求する: SAFE_STOP -> MANUAL_RECOVERY -> NORMAL。
  // 1回の復旧コマンドで即座に自律走行が再開してしまわないようにするための設計。
  if (safety_state_ == SafetyState::SAFE_STOP) {
    RCLCPP_INFO(this->get_logger(), "Recovery command received: SAFE_STOP -> MANUAL_RECOVERY");
    transition_safety_state(SafetyState::MANUAL_RECOVERY);
  } else if (safety_state_ == SafetyState::MANUAL_RECOVERY) {
    RCLCPP_INFO(this->get_logger(), "Recovery command received: MANUAL_RECOVERY -> NORMAL");
    estop_latched_ = false;
    transition_safety_state(SafetyState::NORMAL);
  } else {
    RCLCPP_WARN(
      this->get_logger(), "Recovery command ignored in state %s",
      to_string(safety_state_).c_str());
  }
}

void SafetyStateMachineNode::on_nav2_cmd_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  latest_nav2_cmd_vel_ = msg->twist;
}

void SafetyStateMachineNode::on_sensors_ok(const std_msgs::msg::Bool::SharedPtr msg)
{
  sensors_healthy_ = msg->data;
}

void SafetyStateMachineNode::on_cmd_vel_timer()
{
  geometry_msgs::msg::Twist out;

  switch (safety_state_) {
    case SafetyState::NORMAL:
      out = latest_nav2_cmd_vel_;
      break;
    case SafetyState::DEGRADED:
      out.linear.x = latest_nav2_cmd_vel_.linear.x * degraded_speed_scale_;
      out.linear.y = latest_nav2_cmd_vel_.linear.y * degraded_speed_scale_;
      out.linear.z = latest_nav2_cmd_vel_.linear.z * degraded_speed_scale_;
      out.angular.x = latest_nav2_cmd_vel_.angular.x * degraded_speed_scale_;
      out.angular.y = latest_nav2_cmd_vel_.angular.y * degraded_speed_scale_;
      out.angular.z = latest_nav2_cmd_vel_.angular.z * degraded_speed_scale_;
      break;
    case SafetyState::SAFE_STOP:
    case SafetyState::MANUAL_RECOVERY:
      // センサーが健全(sensors_healthy_)で、かつリモートE-Stop由来のロック
      // (estop_latched_)でなければ、RVizのNav2 Goal含むNav2の指令をそのまま中継する。
      // それ以外はoutをデフォルト構築のゼロ値のままにする(全速度成分0.0)。
      if (sensors_healthy_ && !estop_latched_) {
        out = latest_nav2_cmd_vel_;
      }
      break;
  }

  if (cmd_vel_pub_->is_activated()) {
    geometry_msgs::msg::TwistStamped stamped;
    stamped.header.stamp = this->now();
    stamped.twist = out;
    cmd_vel_pub_->publish(stamped);
  }
}

void SafetyStateMachineNode::on_degraded_timeout()
{
  if (safety_state_ == SafetyState::DEGRADED) {
    RCLCPP_WARN(
      this->get_logger(), "DEGRADED sustained for %ldms -> SAFE_STOP", degraded_timeout_ms_);
    transition_safety_state(SafetyState::SAFE_STOP);
  }
}

void SafetyStateMachineNode::transition_safety_state(SafetyState new_state)
{
  if (new_state == safety_state_) {
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Safety state transition: %s -> %s",
    to_string(safety_state_).c_str(), to_string(new_state).c_str());

  safety_state_ = new_state;
  metrics_.set_gauge(
    "safety_state_machine_state", to_metric_value(safety_state_),
    "Current SafetyState (0=NORMAL, 1=DEGRADED, 2=SAFE_STOP, 3=MANUAL_RECOVERY)");
  metrics_.increment_counter(
    "safety_state_machine_transitions_total{to_state=\"" + to_string(new_state) + "\"}",
    "Number of transitions into each SafetyState");

  if (new_state == SafetyState::DEGRADED) {
    degraded_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(degraded_timeout_ms_),
      std::bind(&SafetyStateMachineNode::on_degraded_timeout, this));
  } else {
    // DEGRADED以外へ遷移したら、猶予タイマーは無効化する
    degraded_timer_.reset();
  }

  if (state_pub_ && state_pub_->is_activated()) {
    std_msgs::msg::String state_msg;
    state_msg.data = to_string(safety_state_);
    state_pub_->publish(state_msg);
  }
}

}  // namespace safety_state_machine
