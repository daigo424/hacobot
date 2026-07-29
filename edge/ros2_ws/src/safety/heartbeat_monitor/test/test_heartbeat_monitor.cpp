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

// heartbeat_monitorのliveliness_lease_msと一致させること(publisher/subscriber双方の
// DDS Liveliness QoSが対称である必要がある)。
constexpr int64_t kLeaseMs = 300;
// SetUp()の200msディスカバリ待ち+最初のpublish確認がliveliness_callbackとして
// 届くまでの余裕を見込む値。この境界を跨ぐと起動猶予チェック(check_startup_liveliness)
// が働くため、各テストの意図した挙動(発行中は異常なし/停止後は検知)を邪魔しない
// 範囲でkLeaseMsより十分大きくしてある。
constexpr int64_t kStartupGraceMs = 500;

class HeartbeatMonitorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      std::vector<rclcpp::Parameter>{
        rclcpp::Parameter("monitored_sources", std::vector<std::string>{"test_source"}),
        rclcpp::Parameter("liveliness_lease_ms", kLeaseMs),
        rclcpp::Parameter("startup_grace_period_ms", kStartupGraceMs),
      });
    node_ = std::make_shared<heartbeat_monitor::HeartbeatMonitorNode>(options);

    helper_node_ = std::make_shared<rclcpp::Node>("test_helper_node");
    const rclcpp::QoS heartbeat_qos = rclcpp::QoS(10)
      .liveliness(rclcpp::LivelinessPolicy::ManualByTopic)
      .liveliness_lease_duration(std::chrono::milliseconds(kLeaseMs));
    heartbeat_pub_ = helper_node_->create_publisher<std_msgs::msg::Empty>(
      "/safety/heartbeat/test_source", heartbeat_qos);
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

TEST_F(HeartbeatMonitorTest, NoAnomalyWhilePublisherIsAsserting)
{
  for (int i = 0; i < 8; ++i) {
    heartbeat_pub_->publish(std_msgs::msg::Empty());
    spin_for(50ms);
  }
  EXPECT_FALSE(anomaly_received_);
}

TEST_F(HeartbeatMonitorTest, DetectsLivelinessLoss)
{
  // 最初は正常にハートビートを送る
  for (int i = 0; i < 3; ++i) {
    heartbeat_pub_->publish(std_msgs::msg::Empty());
    spin_for(50ms);
  }
  EXPECT_FALSE(anomaly_received_);

  // 意図的にハートビートの送信を止め、DDSのlease失効検知(kLeaseMs)を
  // 超えるまで待つ(内部の失効検知の粒度を見込んでマージンを取る)
  spin_for(500ms);

  EXPECT_TRUE(anomaly_received_);
  EXPECT_EQ(last_anomaly_source_, "test_source");
  EXPECT_EQ(last_anomaly_severity_, safety_msgs::msg::AnomalyEvent::WARNING);
}

// recovery_commandを送っても、監視対象からの実際のハートビートが再開しない限り、
// is_stale_フラグがリセットされて即座に再度異常が通知されることを確認する。
// (safety_state_machineだけがNORMALに戻り、検知側が沈黙し続ける既知の限界の回帰テスト)
TEST_F(HeartbeatMonitorTest, RecoveryCommandReDetectsWithoutRealHeartbeat)
{
  // liveliness_callbackはalive->not_aliveの「変化」でしか発火しないため、
  // 一度も生存を主張していないpublisherは変化を観測できない。まず実際に
  // ハートビートを送ってalive状態を成立させてから、停止してlease失効させる。
  for (int i = 0; i < 3; ++i) {
    heartbeat_pub_->publish(std_msgs::msg::Empty());
    spin_for(50ms);
  }
  spin_for(500ms);
  EXPECT_TRUE(anomaly_received_);

  // recovery_commandを送るが、実際のハートビートは再開させない
  anomaly_received_ = false;
  recovery_pub_->publish(std_msgs::msg::Empty());

  // is_stale_がリセットされた直後、currently_alive_がまだfalseのため
  // on_recovery_command()内で即座に再度異常が検知・通知されるはず
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
