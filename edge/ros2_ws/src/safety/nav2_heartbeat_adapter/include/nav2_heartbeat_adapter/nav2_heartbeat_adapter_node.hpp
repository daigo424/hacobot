#ifndef NAV2_HEARTBEAT_ADAPTER__NAV2_HEARTBEAT_ADAPTER_NODE_HPP_
#define NAV2_HEARTBEAT_ADAPTER__NAV2_HEARTBEAT_ADAPTER_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "safety_metrics/prometheus_exporter.hpp"
#include "std_msgs/msg/empty.hpp"

namespace nav2_heartbeat_adapter
{

// Nav2は外部(upstream)パッケージであり、hacobot独自のハートビートプロトコル
// (/safety/heartbeat/<source>)を直接publishさせることができない。
// そのため、Nav2が生きていれば継続的に更新される「代表トピック」
// (デフォルト: /local_costmap/costmap。local_costmapのupdate_frequency=5Hzで
// 常時publishされるため、ナビゲーションゴールの有無に関わらずNav2の生存確認に使える)
// の更新有無を監視し、生きていると判断できる間だけ100ms間隔で
// /safety/heartbeat/nav2 へ変換してpublishするアダプター。
//
// heartbeat_monitorは「/safety/heartbeat/nav2 が届いているか」だけを見るため、
// このアダプターが単にpublishを止めることが、Nav2死活監視の実質的な通知手段になる。
class Nav2HeartbeatAdapterNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit Nav2HeartbeatAdapterNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void on_heartbeat_timer();

  std::string liveness_topic_;
  std::string liveness_topic_type_;
  int64_t liveness_window_ms_;
  int64_t heartbeat_period_ms_;
  int64_t metrics_port_;

  rclcpp::Time last_seen_;

  rclcpp::GenericSubscription::SharedPtr liveness_sub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Empty>> heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  safety_metrics::PrometheusExporter metrics_;
};

}  // namespace nav2_heartbeat_adapter

#endif  // NAV2_HEARTBEAT_ADAPTER__NAV2_HEARTBEAT_ADAPTER_NODE_HPP_
