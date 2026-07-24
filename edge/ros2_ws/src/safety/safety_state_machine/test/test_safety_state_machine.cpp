#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "safety_msgs/msg/anomaly_event.hpp"
#include "safety_state_machine/safety_state_machine_node.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using safety_state_machine::SafetyState;

class SafetyStateMachineTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      std::vector<rclcpp::Parameter>{
        rclcpp::Parameter("degraded_timeout_ms", 200),
        rclcpp::Parameter("cmd_vel_period_ms", 20),
      });
    node_ = std::make_shared<safety_state_machine::SafetyStateMachineNode>(options);

    helper_node_ = std::make_shared<rclcpp::Node>("test_helper_node");
    anomaly_pub_ = helper_node_->create_publisher<safety_msgs::msg::AnomalyEvent>(
      "/safety/anomaly_event", rclcpp::QoS(10));
    recovery_pub_ = helper_node_->create_publisher<std_msgs::msg::Empty>(
      "/safety/recovery_command", rclcpp::QoS(10));
    nav2_cmd_vel_pub_ = helper_node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel_nav2", rclcpp::QoS(10));
    sensors_ok_pub_ = helper_node_->create_publisher<std_msgs::msg::Bool>(
      "/safety/sensors_ok", rclcpp::QoS(1).transient_local());

    cmd_vel_sub_ = helper_node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        last_cmd_vel_ = msg->twist;
        cmd_vel_count_++;
      });

    executor_.add_node(node_->get_node_base_interface());
    executor_.add_node(helper_node_);

    node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    spin_for(200ms);  // ディスカバリ待ち
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

  void publish_anomaly(const std::string & source, const std::string & severity)
  {
    safety_msgs::msg::AnomalyEvent event;
    event.source = source;
    event.severity = severity;
    event.reason = "test-injected";
    anomaly_pub_->publish(event);
  }

  void publish_sensors_ok(bool ok)
  {
    std_msgs::msg::Bool msg;
    msg.data = ok;
    sensors_ok_pub_->publish(msg);
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<safety_state_machine::SafetyStateMachineNode> node_;
  rclcpp::Node::SharedPtr helper_node_;
  rclcpp::Publisher<safety_msgs::msg::AnomalyEvent>::SharedPtr anomaly_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr recovery_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr nav2_cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sensors_ok_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  geometry_msgs::msg::Twist last_cmd_vel_;
  int cmd_vel_count_{0};
};

TEST_F(SafetyStateMachineTest, StartsInNormal)
{
  EXPECT_EQ(node_->current_state(), SafetyState::NORMAL);
}

TEST_F(SafetyStateMachineTest, WarningTransitionsNormalToDegraded)
{
  publish_anomaly("nav2", safety_msgs::msg::AnomalyEvent::WARNING);
  spin_for(100ms);
  EXPECT_EQ(node_->current_state(), SafetyState::DEGRADED);
}

TEST_F(SafetyStateMachineTest, SecondWarningEscalatesDegradedToSafeStop)
{
  publish_anomaly("nav2", safety_msgs::msg::AnomalyEvent::WARNING);
  spin_for(80ms);
  ASSERT_EQ(node_->current_state(), SafetyState::DEGRADED);

  publish_anomaly("comm_bridge", safety_msgs::msg::AnomalyEvent::WARNING);
  spin_for(80ms);
  EXPECT_EQ(node_->current_state(), SafetyState::SAFE_STOP);
}

TEST_F(SafetyStateMachineTest, SustainedDegradedEscalatesToSafeStopAfterTimeout)
{
  publish_anomaly("nav2", safety_msgs::msg::AnomalyEvent::WARNING);
  spin_for(80ms);
  ASSERT_EQ(node_->current_state(), SafetyState::DEGRADED);

  // degraded_timeout_ms=200で上書きしているので、追加のWARNING無しでも
  // 十分待てば自動的にSAFE_STOPへ遷移するはず
  spin_for(300ms);
  EXPECT_EQ(node_->current_state(), SafetyState::SAFE_STOP);
}

TEST_F(SafetyStateMachineTest, CriticalBypassesDegradedFromNormal)
{
  publish_anomaly("lidar", safety_msgs::msg::AnomalyEvent::CRITICAL);
  spin_for(100ms);
  EXPECT_EQ(node_->current_state(), SafetyState::SAFE_STOP);
}

TEST_F(SafetyStateMachineTest, CriticalOverridesDegraded)
{
  publish_anomaly("nav2", safety_msgs::msg::AnomalyEvent::WARNING);
  spin_for(80ms);
  ASSERT_EQ(node_->current_state(), SafetyState::DEGRADED);

  publish_anomaly("lidar", safety_msgs::msg::AnomalyEvent::CRITICAL);
  spin_for(80ms);
  EXPECT_EQ(node_->current_state(), SafetyState::SAFE_STOP);
}

TEST_F(SafetyStateMachineTest, RecoveryRequiresTwoExplicitCommands)
{
  publish_anomaly("lidar", safety_msgs::msg::AnomalyEvent::CRITICAL);
  spin_for(100ms);
  ASSERT_EQ(node_->current_state(), SafetyState::SAFE_STOP);

  recovery_pub_->publish(std_msgs::msg::Empty());
  spin_for(100ms);
  EXPECT_EQ(node_->current_state(), SafetyState::MANUAL_RECOVERY);

  recovery_pub_->publish(std_msgs::msg::Empty());
  spin_for(100ms);
  EXPECT_EQ(node_->current_state(), SafetyState::NORMAL);
}

TEST_F(SafetyStateMachineTest, CmdVelIsZeroedDuringSafeStop)
{
  geometry_msgs::msg::TwistStamped nav2_cmd;
  nav2_cmd.twist.linear.x = 1.0;
  nav2_cmd.twist.angular.z = 0.5;
  nav2_cmd_vel_pub_->publish(nav2_cmd);
  spin_for(60ms);

  // NORMAL中はそのまま中継される
  EXPECT_DOUBLE_EQ(last_cmd_vel_.linear.x, 1.0);

  publish_anomaly("lidar", safety_msgs::msg::AnomalyEvent::CRITICAL);
  spin_for(100ms);
  ASSERT_EQ(node_->current_state(), SafetyState::SAFE_STOP);

  nav2_cmd_vel_pub_->publish(nav2_cmd);
  spin_for(60ms);

  // SAFE_STOP中はNav2からの指令に関わらずゼロにoverrideされる
  EXPECT_DOUBLE_EQ(last_cmd_vel_.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(last_cmd_vel_.angular.z, 0.0);
}

TEST_F(SafetyStateMachineTest, SensorsHealthyDefaultsFalseUntilWatchdogReports)
{
  // watchdogからの通知を一度も受け取っていない起動直後は、安全側(未健全)扱いのはず
  EXPECT_FALSE(node_->sensors_healthy());
}

TEST_F(SafetyStateMachineTest, CmdVelRelayedDuringSafeStopWhenSensorsHealthy)
{
  publish_sensors_ok(true);
  spin_for(60ms);
  ASSERT_TRUE(node_->sensors_healthy());

  // heartbeat_monitor由来のWARNING連続でSAFE_STOPへ(センサー自体は健全という想定)
  publish_anomaly("nav2", safety_msgs::msg::AnomalyEvent::WARNING);
  spin_for(80ms);
  publish_anomaly("comm_bridge", safety_msgs::msg::AnomalyEvent::WARNING);
  spin_for(80ms);
  ASSERT_EQ(node_->current_state(), SafetyState::SAFE_STOP);

  geometry_msgs::msg::TwistStamped nav2_cmd;
  nav2_cmd.twist.linear.x = 0.8;
  nav2_cmd_vel_pub_->publish(nav2_cmd);
  spin_for(60ms);

  // センサーが健全なSAFE_STOPでは、RVizのNav2 Goal等によるcmd_vel_nav2を中継してよい
  EXPECT_DOUBLE_EQ(last_cmd_vel_.linear.x, 0.8);
}

TEST_F(SafetyStateMachineTest, EstopLatchBlocksCmdVelEvenWhenSensorsHealthy)
{
  publish_sensors_ok(true);
  spin_for(60ms);
  ASSERT_TRUE(node_->sensors_healthy());

  publish_anomaly("estop_bridge", safety_msgs::msg::AnomalyEvent::CRITICAL);
  spin_for(80ms);
  ASSERT_EQ(node_->current_state(), SafetyState::SAFE_STOP);
  EXPECT_TRUE(node_->estop_latched());

  geometry_msgs::msg::TwistStamped nav2_cmd;
  nav2_cmd.twist.linear.x = 0.8;
  nav2_cmd_vel_pub_->publish(nav2_cmd);
  spin_for(60ms);

  // センサーが健全でも、リモートE-Stop由来のSAFE_STOPはcmd_velを通さない
  EXPECT_DOUBLE_EQ(last_cmd_vel_.linear.x, 0.0);
}

TEST_F(SafetyStateMachineTest, EstopLatchClearsOnlyAfterFullRecoveryToNormal)
{
  publish_sensors_ok(true);
  spin_for(60ms);

  publish_anomaly("estop_bridge", safety_msgs::msg::AnomalyEvent::CRITICAL);
  spin_for(80ms);
  ASSERT_TRUE(node_->estop_latched());

  recovery_pub_->publish(std_msgs::msg::Empty());
  spin_for(80ms);
  ASSERT_EQ(node_->current_state(), SafetyState::MANUAL_RECOVERY);
  // MANUAL_RECOVERYの間はまだestopロックを保持したまま
  EXPECT_TRUE(node_->estop_latched());

  recovery_pub_->publish(std_msgs::msg::Empty());
  spin_for(80ms);
  ASSERT_EQ(node_->current_state(), SafetyState::NORMAL);
  EXPECT_FALSE(node_->estop_latched());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
