#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "heartbeat_monitor/heartbeat_monitor_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "safety_msgs/msg/anomaly_event.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::chrono_literals;

class HeartbeatMonitorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      std::vector<rclcpp::Parameter>{
        rclcpp::Parameter("monitored_sources", std::vector<std::string>{"test_source"}),
        rclcpp::Parameter("timeout_ms", 300),
        rclcpp::Parameter("check_period_ms", 20),
        // 起動猶予期間のテストはこのテストの対象外なので、通常のtimeout_msと
        // 同じ値にして無効化しておく(そうしないとテストが遅くなる/失敗する)
        rclcpp::Parameter("startup_grace_period_ms", 300),
      });
    node_ = std::make_shared<heartbeat_monitor::HeartbeatMonitorNode>(options);

    helper_node_ = std::make_shared<rclcpp::Node>("test_helper_node");
    heartbeat_pub_ = helper_node_->create_publisher<std_msgs::msg::Empty>(
      "/safety/heartbeat/test_source", rclcpp::QoS(10));
    recovery_pub_ = helper_node_->create_publisher<std_msgs::msg::Empty>(
      "/safety/recovery_command", rclcpp::QoS(10));

    anomaly_received_ = false;
    anomaly_sub_ = helper_node_->create_subscription<safety_msgs::msg::AnomalyEvent>(
      "/safety/anomaly_event", rclcpp::QoS(10),
      [this](const safety_msgs::msg::AnomalyEvent::SharedPtr msg) {
        anomaly_received_ = true;
        last_anomaly_source_ = msg->source;
        last_anomaly_severity_ = msg->severity;
      });

    executor_.add_node(node_->get_node_base_interface());
    executor_.add_node(helper_node_);

    node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    // 購読者・発行者同士のディスカバリが完了するまで少し待つ
    spin_for(200ms);
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

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<heartbeat_monitor::HeartbeatMonitorNode> node_;
  rclcpp::Node::SharedPtr helper_node_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr recovery_pub_;
  rclcpp::Subscription<safety_msgs::msg::AnomalyEvent>::SharedPtr anomaly_sub_;
  bool anomaly_received_;
  std::string last_anomaly_source_;
  std::string last_anomaly_severity_;
};

TEST_F(HeartbeatMonitorTest, NoAnomalyWhileHeartbeatIsPublishing)
{
  for (int i = 0; i < 8; ++i) {
    heartbeat_pub_->publish(std_msgs::msg::Empty());
    spin_for(50ms);
  }
  EXPECT_FALSE(anomaly_received_);
}

TEST_F(HeartbeatMonitorTest, DetectsHeartbeatLoss)
{
  // 最初は正常にハートビートを送る
  for (int i = 0; i < 3; ++i) {
    heartbeat_pub_->publish(std_msgs::msg::Empty());
    spin_for(50ms);
  }
  EXPECT_FALSE(anomaly_received_);

  // 意図的にハートビートの送信を止め、タイムアウト(300ms)を超えるまで待つ
  spin_for(500ms);

  EXPECT_TRUE(anomaly_received_);
  EXPECT_EQ(last_anomaly_source_, "test_source");
  EXPECT_EQ(last_anomaly_severity_, safety_msgs::msg::AnomalyEvent::WARNING);
}

// recovery_commandを送っても、監視対象からの実際のハートビートが再開しない限り、
// is_stale_フラグがリセットされて次のチェック周期で再度異常が通知されることを確認する。
// (safety_state_machineだけがNORMALに戻り、検知側が沈黙し続ける既知の限界の回帰テスト)
TEST_F(HeartbeatMonitorTest, RecoveryCommandAllowsRedetectionWithoutRealHeartbeat)
{
  // タイムアウトさせて最初の異常を検知させる
  spin_for(500ms);
  EXPECT_TRUE(anomaly_received_);

  // recovery_commandを送るが、実際のハートビートは再開させない
  anomaly_received_ = false;
  recovery_pub_->publish(std_msgs::msg::Empty());

  // is_stale_がリセットされ、次のcheck_timeouts()周期で再度異常が検知・通知されるはず
  spin_for(200ms);
  EXPECT_TRUE(anomaly_received_);
  EXPECT_EQ(last_anomaly_source_, "test_source");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
