#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "safety_msgs/msg/anomaly_event.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "slam_toolbox/srv/pause.hpp"
#include "stall_recovery/stall_recovery_node.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace
{
sensor_msgs::msg::LaserScan make_scan(float range_value, size_t n = 20)
{
  sensor_msgs::msg::LaserScan scan;
  scan.range_min = 0.1f;
  scan.range_max = 10.0f;
  scan.ranges.assign(n, range_value);
  return scan;
}

// 円柱等の曲面障害物に押し付けられ、わずかな回転で見え方(全体の並び)は変わっても
// 最も近い1点(押し付けられている箇所)だけは距離がほとんど変わらない状況を模す。
sensor_msgs::msg::LaserScan make_scan_with_pinned_min(
  float min_value, float jitter, size_t n = 20)
{
  sensor_msgs::msg::LaserScan scan;
  scan.range_min = 0.1f;
  scan.range_max = 10.0f;
  scan.ranges.assign(n, 1.0f + jitter);
  scan.ranges[0] = min_value;
  return scan;
}

geometry_msgs::msg::TwistStamped make_cmd(double linear_x, double angular_z = 0.0)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.twist.linear.x = linear_x;
  cmd.twist.angular.z = angular_z;
  return cmd;
}
}  // namespace

class StallRecoveryTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      std::vector<rclcpp::Parameter>{
        rclcpp::Parameter("compare_interval_sec", 0.2),
        rclcpp::Parameter("similarity_threshold", 0.9),
        rclcpp::Parameter("range_epsilon_m", 0.03),
        rclcpp::Parameter("stall_confirm_sec", 0.3),
        rclcpp::Parameter("backoff_duration_sec", 0.2),
        rclcpp::Parameter("backoff_linear_x", -0.08),
        rclcpp::Parameter("turn_duration_sec", 0.3),
        rclcpp::Parameter("turn_angular_z", 0.6),
        rclcpp::Parameter("cooldown_sec", 0.3),
        rclcpp::Parameter("min_linear_cmd", 0.03),
        rclcpp::Parameter("min_angular_cmd", 0.05),
        rclcpp::Parameter("cmd_freshness_sec", 1.0),
        rclcpp::Parameter("repeated_stall_window_sec", 5),
        rclcpp::Parameter("repeated_stall_limit", 2),
        rclcpp::Parameter("control_period_ms", 20),
      });
    node_ = std::make_shared<stall_recovery::StallRecoveryNode>(options);

    helper_node_ = std::make_shared<rclcpp::Node>("test_helper_node");
    scan_pub_ = helper_node_->create_publisher<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS());
    cmd_raw_pub_ = helper_node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel_nav2_raw", rclcpp::QoS(10));
    recovery_pub_ = helper_node_->create_publisher<std_msgs::msg::Empty>(
      "/safety/recovery_command", rclcpp::QoS(10));
    safety_state_pub_ = helper_node_->create_publisher<std_msgs::msg::String>(
      "/safety/state", rclcpp::QoS(1).transient_local());

    cmd_out_sub_ = helper_node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel_nav2", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        last_cmd_out_ = msg;
        cmd_out_count_++;
      });
    anomaly_sub_ = helper_node_->create_subscription<safety_msgs::msg::AnomalyEvent>(
      "/safety/anomaly_event", rclcpp::QoS(10),
      [this](const safety_msgs::msg::AnomalyEvent::SharedPtr msg) {
        anomaly_received_ = true;
        last_anomaly_source_ = msg->source;
        last_anomaly_severity_ = msg->severity;
      });

    // slam_toolboxのpause_new_measurements(トグル式)を模した偽サービス。
    // 呼ばれるたびにpaused_をトグルし、呼び出し回数を記録する。
    pause_service_ = helper_node_->create_service<slam_toolbox::srv::Pause>(
      "slam_toolbox/pause_new_measurements",
      [this](
        const std::shared_ptr<slam_toolbox::srv::Pause::Request>/*request*/,
        std::shared_ptr<slam_toolbox::srv::Pause::Response> response) {
        pause_service_call_count_++;
        paused_ = !paused_;
        response->status = paused_;
      });

    executor_.add_node(node_->get_node_base_interface());
    executor_.add_node(helper_node_);

    node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    spin_for(200ms);  // ディスカバリ待ち

    // 既定ではNORMAL相当(cmd_velが中継される状態)とみなす。SAFE_STOP等を
    // 検証したいテストは個別にpublishし直す。
    std_msgs::msg::String state_msg;
    state_msg.data = "NORMAL";
    safety_state_pub_->publish(state_msg);
    spin_for(100ms);
  }

  void TearDown() override
  {
    node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
    executor_.remove_node(node_->get_node_base_interface());
    executor_.remove_node(helper_node_);
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      executor_.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }

  // 「壁にスタックしてホイールが空転している」状況を模して、変化しないスキャンと
  // 前進コマンドを一定時間publishし続ける
  void publish_stall_scenario(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      scan_pub_->publish(make_scan(1.0f));
      cmd_raw_pub_->publish(make_cmd(0.2));
      spin_for(30ms);
    }
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<stall_recovery::StallRecoveryNode> node_;
  rclcpp::Node::SharedPtr helper_node_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr recovery_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_out_sub_;
  rclcpp::Subscription<safety_msgs::msg::AnomalyEvent>::SharedPtr anomaly_sub_;
  rclcpp::Service<slam_toolbox::srv::Pause>::SharedPtr pause_service_;

  geometry_msgs::msg::TwistStamped::SharedPtr last_cmd_out_;
  int cmd_out_count_{0};
  bool anomaly_received_{false};
  std::string last_anomaly_source_;
  std::string last_anomaly_severity_;
  int pause_service_call_count_{0};
  bool paused_{false};
};

TEST_F(StallRecoveryTest, RelaysCommandDirectlyWhenScanIsChanging)
{
  float range = 1.0f;
  for (int i = 0; i < 15; ++i) {
    scan_pub_->publish(make_scan(range));
    range += 0.15f;  // 実際に動いている場合を模して毎回大きく変化させる
    cmd_raw_pub_->publish(make_cmd(0.2));
    spin_for(30ms);
  }

  ASSERT_EQ(node_->current_mode(), stall_recovery::StallRecoveryNode::Mode::PASSTHROUGH);
  ASSERT_TRUE(last_cmd_out_ != nullptr);
  EXPECT_NEAR(last_cmd_out_->twist.linear.x, 0.2, 1e-6);
  EXPECT_FALSE(anomaly_received_);
}

TEST_F(StallRecoveryTest, NoStallWhenNotCommandingMotion)
{
  const auto end = std::chrono::steady_clock::now() + 1200ms;
  while (std::chrono::steady_clock::now() < end) {
    scan_pub_->publish(make_scan(1.0f));
    cmd_raw_pub_->publish(make_cmd(0.0));
    spin_for(30ms);
  }

  EXPECT_EQ(node_->current_mode(), stall_recovery::StallRecoveryNode::Mode::PASSTHROUGH);
}

// safety_state_machineがSAFE_STOP/MANUAL_RECOVERY中はcmd_velを強制的にゼロにしており、
// その間はNav2が指令を出し続けていてもロボットは実際には動かない。これをホイール
// 空転による本物のスタックと誤検知しないことを確認する。
TEST_F(StallRecoveryTest, NoStallWhileSafetyStateIsNotRelayingCmdVel)
{
  std_msgs::msg::String state_msg;
  state_msg.data = "SAFE_STOP";
  safety_state_pub_->publish(state_msg);
  spin_for(100ms);

  const auto end = std::chrono::steady_clock::now() + 1200ms;
  while (std::chrono::steady_clock::now() < end) {
    scan_pub_->publish(make_scan(1.0f));  // 全く変化しないスキャン
    cmd_raw_pub_->publish(make_cmd(0.2));  // Nav2はまだ指令を出し続けている想定
    spin_for(30ms);
  }

  EXPECT_EQ(node_->current_mode(), stall_recovery::StallRecoveryNode::Mode::PASSTHROUGH);
  EXPECT_FALSE(anomaly_received_);
}

TEST_F(StallRecoveryTest, DetectsStallAndBacksOffThenTurnsThenReturnsToPassthrough)
{
  bool saw_backoff_twist = false;
  bool saw_turn_twist = false;
  const auto end = std::chrono::steady_clock::now() + 1200ms;
  while (std::chrono::steady_clock::now() < end) {
    scan_pub_->publish(make_scan(1.0f));  // 全く変化しないスキャン(ホイール空転を模す)
    cmd_raw_pub_->publish(make_cmd(0.2));
    spin_for(30ms);
    if (last_cmd_out_ && last_cmd_out_->twist.linear.x < 0.0 &&
      last_cmd_out_->twist.angular.z == 0.0)
    {
      saw_backoff_twist = true;
    }
    if (last_cmd_out_ && std::abs(last_cmd_out_->twist.angular.z) > 0.0 &&
      last_cmd_out_->twist.linear.x == 0.0)
    {
      saw_turn_twist = true;
    }
  }

  EXPECT_TRUE(saw_backoff_twist);
  EXPECT_TRUE(saw_turn_twist);

  // backoff(0.2s) + turn(0.3s) + cooldown(0.3s)を待てばPASSTHROUGHへ戻るはず
  spin_for(900ms);
  EXPECT_EQ(node_->current_mode(), stall_recovery::StallRecoveryNode::Mode::PASSTHROUGH);
}

// スタック検知の瞬間にslam_toolboxのpause_new_measurementsを呼び、
// PASSTHROUGH復帰時にもう一度呼んで再開することを確認する
TEST_F(StallRecoveryTest, PausesAndResumesSlamAroundStall)
{
  // pause呼び出しが起きた瞬間を捉える(turn_duration+cooldownが短いテスト設定では
  // 固定時間待ってから確認すると既に再開後になってしまうことがあるため)
  const auto end = std::chrono::steady_clock::now() + 900ms;
  while (std::chrono::steady_clock::now() < end && pause_service_call_count_ == 0) {
    scan_pub_->publish(make_scan(1.0f));
    cmd_raw_pub_->publish(make_cmd(0.2));
    spin_for(30ms);
  }

  ASSERT_GE(pause_service_call_count_, 1);
  EXPECT_TRUE(paused_);

  // backoff(0.2s) + turn(0.3s) + cooldown(0.3s)を待てばPASSTHROUGHへ戻り、再開されるはず
  spin_for(900ms);
  ASSERT_EQ(node_->current_mode(), stall_recovery::StallRecoveryNode::Mode::PASSTHROUGH);
  EXPECT_GE(pause_service_call_count_, 2);
  EXPECT_FALSE(paused_);
}

TEST_F(StallRecoveryTest, DetectsStallAgainstCurvedObstacleViaMinRange)
{
  bool saw_turn_twist = false;
  float jitter = 0.0f;
  const auto end = std::chrono::steady_clock::now() + 900ms;
  while (std::chrono::steady_clock::now() < end) {
    // 全体の並びは毎回揺らす(曲面での回転を模す)が、最も近い点(index 0)は
    // 障害物にほぼ密着したまま(0.15m, ほぼ不変)にする
    scan_pub_->publish(make_scan_with_pinned_min(0.15f, jitter));
    jitter += 0.3f;
    cmd_raw_pub_->publish(make_cmd(0.2));
    spin_for(30ms);
    if (last_cmd_out_ && std::abs(last_cmd_out_->twist.angular.z) > 0.0 &&
      last_cmd_out_->twist.linear.x == 0.0)
    {
      saw_turn_twist = true;
    }
  }

  EXPECT_TRUE(saw_turn_twist);
}

// 直前の離脱動作の直後(consecutive_stall_gap_sec_以内)に、堅牢な検知ロジック
// (指令通り動いているのにスキャンが変化しない)が再びスタックを確認した場合は、
// 後退を挟まず旋回だけを行うことを確認する(毎回後退すると、旋回では向きを
// 変えきれない障害物の前で後退と旋回を無限に繰り返しかねないため)。
// 「近くに何かある」という距離だけでの再判定(誤検知の原因になった)は使わない。
TEST_F(StallRecoveryTest, SkipsBackoffOnConsecutiveStallShortlyAfterRecovery)
{
  bool saw_backoff_twist = false;
  int backoff_phase_entries = 0;
  int turn_phase_entries = 0;
  auto last_mode = node_->current_mode();
  float jitter = 0.0f;

  const auto end = std::chrono::steady_clock::now() + 2500ms;
  while (std::chrono::steady_clock::now() < end) {
    // 障害物までの最小距離が常に近接(0.15m)なままの状況を再現する
    scan_pub_->publish(make_scan_with_pinned_min(0.15f, jitter));
    jitter += 0.3f;
    cmd_raw_pub_->publish(make_cmd(0.2));
    spin_for(30ms);

    if (last_cmd_out_ && last_cmd_out_->twist.linear.x < 0.0) {
      saw_backoff_twist = true;
    }
    const auto mode = node_->current_mode();
    if (mode == stall_recovery::StallRecoveryNode::Mode::BACKING_OFF &&
      last_mode != stall_recovery::StallRecoveryNode::Mode::BACKING_OFF)
    {
      backoff_phase_entries++;
    }
    if (mode == stall_recovery::StallRecoveryNode::Mode::TURNING &&
      last_mode != stall_recovery::StallRecoveryNode::Mode::TURNING)
    {
      turn_phase_entries++;
    }
    last_mode = mode;
  }

  EXPECT_TRUE(saw_backoff_twist);
  // 最初の1回だけ後退し、その後は繰り返しスタックしても後退を挟まないはず
  EXPECT_EQ(backoff_phase_entries, 1);
  EXPECT_GE(turn_phase_entries, 2);
}

TEST_F(StallRecoveryTest, EscalatesToCriticalAfterRepeatedStalls)
{
  // repeated_stall_limit=2なので、3回目のスタックでCRITICALへエスカレーションするはず。
  // 1サイクルはstall_confirm+backoff+turn+cooldownで約1.3秒かかるため余裕を持たせる。
  for (int cycle = 0; cycle < 3; ++cycle) {
    publish_stall_scenario(1400ms);
  }

  EXPECT_EQ(node_->current_mode(), stall_recovery::StallRecoveryNode::Mode::ESCALATED);
  EXPECT_TRUE(anomaly_received_);
  EXPECT_EQ(last_anomaly_source_, "stall_recovery");
  EXPECT_EQ(last_anomaly_severity_, safety_msgs::msg::AnomalyEvent::CRITICAL);
}

TEST_F(StallRecoveryTest, RecoveryCommandResumesMonitoringAfterEscalation)
{
  for (int cycle = 0; cycle < 3; ++cycle) {
    publish_stall_scenario(1400ms);
  }
  ASSERT_EQ(node_->current_mode(), stall_recovery::StallRecoveryNode::Mode::ESCALATED);

  recovery_pub_->publish(std_msgs::msg::Empty());
  spin_for(200ms);

  EXPECT_EQ(node_->current_mode(), stall_recovery::StallRecoveryNode::Mode::PASSTHROUGH);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
