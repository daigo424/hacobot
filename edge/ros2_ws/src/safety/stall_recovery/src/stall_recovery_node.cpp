#include "stall_recovery/stall_recovery_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace stall_recovery
{

std::string StallRecoveryNode::to_string(Mode mode)
{
  switch (mode) {
    case Mode::PASSTHROUGH: return "PASSTHROUGH";
    case Mode::BACKING_OFF: return "BACKING_OFF";
    case Mode::TURNING: return "TURNING";
    case Mode::COOLDOWN: return "COOLDOWN";
    case Mode::ESCALATED: return "ESCALATED";
  }
  return "UNKNOWN";
}

StallRecoveryNode::StallRecoveryNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("stall_recovery", options),
  compare_interval_sec_(1.0),
  similarity_threshold_(0.9),
  range_epsilon_m_(0.03),
  // 0.25mだと既に接触・密着した状態でしか検知できず、特に円柱等の回転対称な
  // 障害物は密着後に旋回しても中心からの距離が変わらず抜け出せない(実測で確認)。
  // 接触前、まだ余裕がある段階で検知できるよう広げる。
  near_obstacle_threshold_m_(0.45),
  stall_confirm_sec_(1.5),
  backoff_duration_sec_(0.6),
  backoff_linear_x_(-0.12),
  turn_duration_sec_(1.5),
  turn_angular_z_(0.6),
  cooldown_sec_(2.0),
  consecutive_stall_gap_sec_(3.0),
  min_linear_cmd_(0.03),
  min_angular_cmd_(0.05),
  cmd_freshness_sec_(1.0),
  // 自力脱出は毎回成功するため、7回(約1回転)でのエスカレーションは早すぎる。
  // 12回(13回目でエスカレーション)、window300秒(検知間隔8〜18秒に余裕を持たせる)へ緩和。
  repeated_stall_window_sec_(300),
  repeated_stall_limit_(12),
  control_period_ms_(50),
  metrics_port_(9106),
  mode_(Mode::PASSTHROUGH),
  stall_timer_running_(false),
  slam_paused_(false)
{
  this->declare_parameter<double>("compare_interval_sec", compare_interval_sec_);
  this->declare_parameter<double>("similarity_threshold", similarity_threshold_);
  this->declare_parameter<double>("range_epsilon_m", range_epsilon_m_);
  this->declare_parameter<double>("near_obstacle_threshold_m", near_obstacle_threshold_m_);
  this->declare_parameter<double>("stall_confirm_sec", stall_confirm_sec_);
  this->declare_parameter<double>("backoff_duration_sec", backoff_duration_sec_);
  this->declare_parameter<double>("backoff_linear_x", backoff_linear_x_);
  this->declare_parameter<double>("turn_duration_sec", turn_duration_sec_);
  this->declare_parameter<double>("turn_angular_z", turn_angular_z_);
  this->declare_parameter<double>("cooldown_sec", cooldown_sec_);
  this->declare_parameter<double>("consecutive_stall_gap_sec", consecutive_stall_gap_sec_);
  this->declare_parameter<double>("min_linear_cmd", min_linear_cmd_);
  this->declare_parameter<double>("min_angular_cmd", min_angular_cmd_);
  this->declare_parameter<double>("cmd_freshness_sec", cmd_freshness_sec_);
  this->declare_parameter<int64_t>("repeated_stall_window_sec", repeated_stall_window_sec_);
  this->declare_parameter<int64_t>("repeated_stall_limit", repeated_stall_limit_);
  this->declare_parameter<int64_t>("control_period_ms", control_period_ms_);
  this->declare_parameter<int64_t>("metrics_port", metrics_port_);
}

StallRecoveryNode::CallbackReturn StallRecoveryNode::on_configure(
  const rclcpp_lifecycle::State & /*state*/)
{
  compare_interval_sec_ = this->get_parameter("compare_interval_sec").as_double();
  similarity_threshold_ = this->get_parameter("similarity_threshold").as_double();
  range_epsilon_m_ = this->get_parameter("range_epsilon_m").as_double();
  near_obstacle_threshold_m_ = this->get_parameter("near_obstacle_threshold_m").as_double();
  stall_confirm_sec_ = this->get_parameter("stall_confirm_sec").as_double();
  backoff_duration_sec_ = this->get_parameter("backoff_duration_sec").as_double();
  backoff_linear_x_ = this->get_parameter("backoff_linear_x").as_double();
  turn_duration_sec_ = this->get_parameter("turn_duration_sec").as_double();
  turn_angular_z_ = this->get_parameter("turn_angular_z").as_double();
  cooldown_sec_ = this->get_parameter("cooldown_sec").as_double();
  consecutive_stall_gap_sec_ = this->get_parameter("consecutive_stall_gap_sec").as_double();
  min_linear_cmd_ = this->get_parameter("min_linear_cmd").as_double();
  min_angular_cmd_ = this->get_parameter("min_angular_cmd").as_double();
  cmd_freshness_sec_ = this->get_parameter("cmd_freshness_sec").as_double();
  repeated_stall_window_sec_ = this->get_parameter("repeated_stall_window_sec").as_int();
  repeated_stall_limit_ = this->get_parameter("repeated_stall_limit").as_int();
  control_period_ms_ = this->get_parameter("control_period_ms").as_int();
  metrics_port_ = this->get_parameter("metrics_port").as_int();
  metrics_.start(static_cast<int>(metrics_port_));

  mode_ = Mode::PASSTHROUGH;
  reference_scan_.reset();
  latest_cmd_.reset();
  stall_timer_running_ = false;
  recent_stall_times_.clear();
  last_recovery_end_time_.reset();
  safety_state_.clear();

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/cmd_vel_nav2", rclcpp::QoS(10));
  anomaly_pub_ = this->create_publisher<safety_msgs::msg::AnomalyEvent>(
    "/safety/anomaly_event", rclcpp::QoS(10));
  mode_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/safety/stall_recovery_mode", rclcpp::QoS(1).transient_local());
  explore_resume_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "explore/resume", rclcpp::QoS(10));
  pause_client_ = this->create_client<slam_toolbox::srv::Pause>(
    "slam_toolbox/pause_new_measurements");
  slam_paused_ = false;

  RCLCPP_INFO(this->get_logger(), "Configured");
  return CallbackReturn::SUCCESS;
}

StallRecoveryNode::CallbackReturn StallRecoveryNode::on_activate(
  const rclcpp_lifecycle::State & /*state*/)
{
  cmd_vel_pub_->on_activate();
  anomaly_pub_->on_activate();
  mode_pub_->on_activate();
  explore_resume_pub_->on_activate();

  const auto now = this->now();
  mode_ = Mode::PASSTHROUGH;
  mode_change_time_ = now;
  reference_scan_.reset();
  reference_scan_time_ = now;
  latest_cmd_.reset();
  latest_cmd_time_ = now;
  stall_timer_running_ = false;

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&StallRecoveryNode::on_scan, this, std::placeholders::_1));
  cmd_vel_raw_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/cmd_vel_nav2_raw", rclcpp::QoS(10),
    std::bind(&StallRecoveryNode::on_cmd_vel_raw, this, std::placeholders::_1));
  recovery_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "/safety/recovery_command", rclcpp::QoS(10),
    std::bind(&StallRecoveryNode::on_recovery_command, this, std::placeholders::_1));
  safety_state_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/safety/state", rclcpp::QoS(1).transient_local(),
    std::bind(&StallRecoveryNode::on_safety_state, this, std::placeholders::_1));

  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(control_period_ms_),
    std::bind(&StallRecoveryNode::on_control_timer, this));

  publish_mode();

  RCLCPP_INFO(this->get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

StallRecoveryNode::CallbackReturn StallRecoveryNode::on_deactivate(
  const rclcpp_lifecycle::State & /*state*/)
{
  control_timer_.reset();
  scan_sub_.reset();
  cmd_vel_raw_sub_.reset();
  recovery_sub_.reset();
  safety_state_sub_.reset();
  cmd_vel_pub_->on_deactivate();
  anomaly_pub_->on_deactivate();
  mode_pub_->on_deactivate();
  explore_resume_pub_->on_deactivate();

  RCLCPP_INFO(this->get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

StallRecoveryNode::CallbackReturn StallRecoveryNode::on_cleanup(
  const rclcpp_lifecycle::State & /*state*/)
{
  cmd_vel_pub_.reset();
  anomaly_pub_.reset();
  mode_pub_.reset();
  pause_client_.reset();
  reference_scan_.reset();
  latest_cmd_.reset();
  recent_stall_times_.clear();
  metrics_.stop();

  RCLCPP_INFO(this->get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

StallRecoveryNode::CallbackReturn StallRecoveryNode::on_shutdown(
  const rclcpp_lifecycle::State & /*state*/)
{
  control_timer_.reset();
  scan_sub_.reset();
  cmd_vel_raw_sub_.reset();
  recovery_sub_.reset();
  safety_state_sub_.reset();
  cmd_vel_pub_.reset();
  anomaly_pub_.reset();
  mode_pub_.reset();
  pause_client_.reset();
  metrics_.stop();

  RCLCPP_INFO(this->get_logger(), "Shutdown");
  return CallbackReturn::SUCCESS;
}

bool StallRecoveryNode::is_commanding_motion() const
{
  if (!latest_cmd_) {
    return false;
  }
  const auto elapsed = (this->now() - latest_cmd_time_).seconds();
  if (elapsed > cmd_freshness_sec_) {
    return false;
  }
  const auto & twist = latest_cmd_->twist;
  return std::abs(twist.linear.x) > min_linear_cmd_ ||
         std::abs(twist.angular.z) > min_angular_cmd_;
}

double StallRecoveryNode::compute_similarity(
  const sensor_msgs::msg::LaserScan & a, const sensor_msgs::msg::LaserScan & b) const
{
  const auto n = std::min(a.ranges.size(), b.ranges.size());
  int64_t valid_count = 0;
  int64_t close_count = 0;
  for (size_t i = 0; i < n; ++i) {
    const float ra = a.ranges[i];
    const float rb = b.ranges[i];
    const bool a_valid = std::isfinite(ra) && ra >= a.range_min && ra <= a.range_max;
    const bool b_valid = std::isfinite(rb) && rb >= b.range_min && rb <= b.range_max;
    if (!a_valid || !b_valid) {
      continue;
    }
    ++valid_count;
    if (std::abs(ra - rb) < range_epsilon_m_) {
      ++close_count;
    }
  }
  // 有効な比較点が無ければ判定根拠が無いため、スタック扱いしない安全側(0.0)を返す
  if (valid_count == 0) {
    return 0.0;
  }
  return static_cast<double>(close_count) / static_cast<double>(valid_count);
}

double StallRecoveryNode::compute_min_range(const sensor_msgs::msg::LaserScan & s)
{
  double min_r = std::numeric_limits<double>::infinity();
  for (const float r : s.ranges) {
    if (std::isfinite(r) && r >= s.range_min && r <= s.range_max) {
      min_r = std::min(min_r, static_cast<double>(r));
    }
  }
  return min_r;
}

void StallRecoveryNode::on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  const auto now = this->now();

  if (!reference_scan_ || (now - reference_scan_time_).seconds() >= compare_interval_sec_) {
    reference_scan_ = msg;
    reference_scan_time_ = now;
    return;
  }

  if (mode_ != Mode::PASSTHROUGH) {
    return;
  }

  const double similarity = compute_similarity(*msg, *reference_scan_);
  metrics_.set_gauge(
    "stall_recovery_scan_similarity", similarity,
    "Fraction of valid LaserScan points nearly unchanged vs the reference scan");

  // 円柱等の曲面障害物では、わずかな回転だけでも見え方(スキャン全体の形)が
  // 大きく変わり、similarityによる検知をすり抜けることがある。障害物の形状に
  // 依存しない補助指標として、最小距離(直近の障害物までの距離)が近接かつ
  // ほぼ一定のまま張り付いていないかも併せて見る。
  const double min_now = compute_min_range(*msg);
  const double min_ref = compute_min_range(*reference_scan_);
  const bool min_range_stuck =
    std::isfinite(min_now) && std::isfinite(min_ref) &&
    min_now < near_obstacle_threshold_m_ &&
    std::abs(min_now - min_ref) < range_epsilon_m_;

  // safety_state_machineがSAFE_STOP/MANUAL_RECOVERY中はcmd_velを強制的にゼロにしており、
  // Nav2が指令を出し続けていてもロボットは実際には動かない。これをホイール空転と
  // 誤検知しないよう、実際にcmd_velが中継される状態でのみ検知する。
  const bool cmd_vel_is_relayed = safety_state_ == "NORMAL" || safety_state_ == "DEGRADED";

  if (cmd_vel_is_relayed && is_commanding_motion() &&
    (similarity >= similarity_threshold_ || min_range_stuck))
  {
    if (!stall_timer_running_) {
      stall_since_ = now;
      stall_timer_running_ = true;
    } else if ((now - stall_since_).seconds() >= stall_confirm_sec_) {
      enter_backing_off();
    }
  } else {
    stall_timer_running_ = false;
  }
}

void StallRecoveryNode::on_cmd_vel_raw(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  latest_cmd_ = msg;
  latest_cmd_time_ = this->now();

  if ((mode_ == Mode::PASSTHROUGH || mode_ == Mode::ESCALATED) && cmd_vel_pub_->is_activated()) {
    cmd_vel_pub_->publish(*msg);
  }
}

void StallRecoveryNode::on_recovery_command(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  // ESCALATED(繰り返しスタックによりCRITICALへエスカレーション済み)のままだと
  // on_scan()がスタック検知自体を止めてしまうため、人手復旧を機に監視を再開する。
  if (mode_ == Mode::ESCALATED) {
    recent_stall_times_.clear();
    // ESCALATED経由の復旧は「後退を省略して良いほど直近」とはみなさない
    last_recovery_end_time_.reset();
    enter_passthrough();
    RCLCPP_INFO(this->get_logger(), "Recovery command received; resuming stall monitoring");
  }
}

void StallRecoveryNode::on_safety_state(const std_msgs::msg::String::SharedPtr msg)
{
  safety_state_ = msg->data;
}

void StallRecoveryNode::on_control_timer()
{
  const auto now = this->now();

  if (mode_ == Mode::BACKING_OFF) {
    if ((now - mode_change_time_).seconds() >= backoff_duration_sec_) {
      enter_turning();
      return;
    }
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now;
    cmd.twist.linear.x = backoff_linear_x_;
    if (cmd_vel_pub_->is_activated()) {
      cmd_vel_pub_->publish(cmd);
    }
  } else if (mode_ == Mode::TURNING) {
    if ((now - mode_change_time_).seconds() >= turn_duration_sec_) {
      enter_cooldown();
      return;
    }
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now;
    cmd.twist.angular.z = turn_angular_z_;
    if (cmd_vel_pub_->is_activated()) {
      cmd_vel_pub_->publish(cmd);
    }
  } else if (mode_ == Mode::COOLDOWN) {
    if ((now - mode_change_time_).seconds() >= cooldown_sec_) {
      last_recovery_end_time_ = now;
      enter_passthrough();
      return;
    }
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now;
    if (cmd_vel_pub_->is_activated()) {
      cmd_vel_pub_->publish(cmd);
    }
  }
}

bool StallRecoveryNode::record_stall_and_maybe_escalate()
{
  const auto now = this->now();
  recent_stall_times_.push_back(now);
  while (!recent_stall_times_.empty() &&
    (now - recent_stall_times_.front()).seconds() > static_cast<double>(repeated_stall_window_sec_))
  {
    recent_stall_times_.pop_front();
  }

  if (static_cast<int64_t>(recent_stall_times_.size()) > repeated_stall_limit_) {
    escalate(
      "stall recovery repeated " + std::to_string(recent_stall_times_.size()) +
      " times within " + std::to_string(repeated_stall_window_sec_) + "s; needs operator attention");
    return true;
  }
  return false;
}

void StallRecoveryNode::enter_backing_off()
{
  stall_timer_running_ = false;
  // 通常の後退→旋回パス・後退省略パス・エスカレーションのいずれに進む場合も、
  // ここが必ず最初に呼ばれるため、1箇所でexplore_liteへの制御奪取宣言を出せる。
  publish_explore_resume(false);

  if (record_stall_and_maybe_escalate()) {
    return;
  }

  const auto now = this->now();
  // 直前の離脱動作の直後(consecutive_stall_gap_sec_以内)に同じ堅牢な判定で
  // 再びスタックが確認された場合は、既に後退で隙間を作った直後とみなし
  // 後退を省略してそのまま旋回する。
  if (last_recovery_end_time_ &&
    (now - *last_recovery_end_time_).seconds() < consecutive_stall_gap_sec_)
  {
    metrics_.increment_counter(
      "stall_recovery_turn_total",
      "Number of times a stall was detected and an automatic in-place turn was triggered");
    RCLCPP_WARN(
      this->get_logger(),
      "Stall detected again shortly after recovery; turning in place for %.1fs (skipping backoff)",
      turn_duration_sec_);
    set_slam_paused(true);
    enter_turning();
    return;
  }

  mode_ = Mode::BACKING_OFF;
  mode_change_time_ = now;
  metrics_.increment_counter(
    "stall_recovery_turn_total",
    "Number of times a stall was detected and an automatic in-place turn was triggered");
  RCLCPP_WARN(
    this->get_logger(),
    "Stall detected (commanded motion but scan unchanged); backing off for %.1fs then turning in place for %.1fs",
    backoff_duration_sec_, turn_duration_sec_);
  set_slam_paused(true);
  publish_mode();
}

void StallRecoveryNode::enter_turning()
{
  mode_ = Mode::TURNING;
  mode_change_time_ = this->now();
  publish_mode();
}

void StallRecoveryNode::enter_cooldown()
{
  mode_ = Mode::COOLDOWN;
  mode_change_time_ = this->now();
  publish_mode();
}

void StallRecoveryNode::enter_passthrough()
{
  mode_ = Mode::PASSTHROUGH;
  mode_change_time_ = this->now();
  stall_timer_running_ = false;
  // 旋回動作そのものでスキャンが変化するため、直前の基準スキャンと比べると
  // 誤って"動いた"と判定される。次のスキャンを新しい基準として取り直す。
  reference_scan_.reset();
  if (slam_paused_) {
    set_slam_paused(false);
  }
  // 正常なCOOLDOWN終了・ESCALATEDからの人手復旧のどちらもここを通るため、
  // 1箇所でexplore_liteへの制御返却宣言を出せる。
  publish_explore_resume(true);
  publish_mode();
}

void StallRecoveryNode::publish_mode()
{
  if (!mode_pub_ || !mode_pub_->is_activated()) {
    return;
  }
  std_msgs::msg::String msg;
  // stallが何回目か(repeated_stall_limit_を超えるとescalate)を併記し、
  // モード名だけでは分からない「エスカレーションまでの余裕」を可視化する。
  msg.data = to_string(mode_) + " [stall " +
    std::to_string(recent_stall_times_.size()) + "/" +
    std::to_string(repeated_stall_limit_) + "]";
  mode_pub_->publish(msg);
}

void StallRecoveryNode::publish_explore_resume(bool resume)
{
  if (!explore_resume_pub_ || !explore_resume_pub_->is_activated()) {
    return;
  }
  std_msgs::msg::Bool msg;
  msg.data = resume;
  explore_resume_pub_->publish(msg);
}

void StallRecoveryNode::set_slam_paused(bool paused)
{
  if (!pause_client_->service_is_ready()) {
    RCLCPP_WARN(
      this->get_logger(),
      "slam_toolbox/pause_new_measurements service not available; cannot %s mapping",
      paused ? "pause" : "resume");
    return;
  }
  // Pauseはトグル式(要求フィールドが無い)なので、現在の意図(slam_paused_)と
  // ズレないよう、呼び出し側でしか状態を管理できない。レスポンスのstatusは
  // 常にtrue(呼び出し成功)を返すだけでトグル後の状態は表さないため使わない。
  slam_paused_ = paused;
  auto request = std::make_shared<slam_toolbox::srv::Pause::Request>();
  pause_client_->async_send_request(
    request,
    [this, paused](rclcpp::Client<slam_toolbox::srv::Pause>::SharedFuture /*future*/) {
      RCLCPP_INFO(
        this->get_logger(), "slam_toolbox mapping %s", paused ? "paused" : "resumed");
    });
}

void StallRecoveryNode::escalate(const std::string & reason)
{
  mode_ = Mode::ESCALATED;
  mode_change_time_ = this->now();
  publish_mode();

  safety_msgs::msg::AnomalyEvent event;
  event.stamp = this->now();
  event.source = "stall_recovery";
  event.severity = safety_msgs::msg::AnomalyEvent::CRITICAL;
  event.reason = reason;
  if (anomaly_pub_->is_activated()) {
    anomaly_pub_->publish(event);
  }
  metrics_.increment_counter(
    "stall_recovery_escalation_total",
    "Number of times repeated stalls were escalated to a CRITICAL anomaly event");
  RCLCPP_ERROR(this->get_logger(), "Escalating to CRITICAL: %s", reason.c_str());
}

}  // namespace stall_recovery
