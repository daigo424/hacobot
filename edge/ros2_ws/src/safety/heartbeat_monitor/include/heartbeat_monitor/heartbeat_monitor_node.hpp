#ifndef HEARTBEAT_MONITOR__HEARTBEAT_MONITOR_NODE_HPP_
#define HEARTBEAT_MONITOR__HEARTBEAT_MONITOR_NODE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "safety_msgs/msg/anomaly_event.hpp"
#include "safety_metrics/prometheus_exporter.hpp"
#include "std_msgs/msg/empty.hpp"

namespace heartbeat_monitor
{

// 監視対象ノード(Nav2, センサードライバ, 通信ブリッジ等)が /safety/heartbeat/<source> へ
// 一定間隔(想定100ms)でpublishするハートビートの受信タイムスタンプを追跡し、
// 一定時間(デフォルト300ms)途絶したら
// /safety/anomaly_event に異常イベントをpublishするlifecycle node。
//
// on_activate()直後は監視対象(Nav2等)側もまだ起動中でハートビートが届いていない
// ことが普通にあるため、startup_grace_period_ms(既定値はtimeout_msより大きい)の間だけ
// 通常のtimeout_msより緩い閾値を適用し、システム起動直後の誤検知を防ぐ。
//
// /safety/recovery_commandを受信すると、is_stale_フラグ(異常を既に通知済みかどうか)を
// 全監視対象についてリセットする。これによりsafety_state_machineがNORMALへ復旧した後、
// 実際には監視対象が復旧していなければ次のcheck_timeouts()周期で即座に再度異常を
// publishできる(is_stale_をリセットしないと1回検知した対象は二度と再通知されないため)。
// last_heartbeat_はここでは更新しない -- 実際に新しいハートビートが届いた場合のみ
// 正常とみなす、フェイルセーフ優先の設計とするため。
class HeartbeatMonitorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit HeartbeatMonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void check_timeouts();
  void on_recovery_command(const std_msgs::msg::Empty::SharedPtr msg);

  std::vector<std::string> monitored_sources_;
  int64_t timeout_ms_;
  int64_t check_period_ms_;
  int64_t startup_grace_period_ms_;
  int64_t metrics_port_;

  rclcpp::Time activated_at_;
  std::map<std::string, rclcpp::Time> last_heartbeat_;
  std::map<std::string, bool> is_stale_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr> heartbeat_subs_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr recovery_sub_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<safety_msgs::msg::AnomalyEvent>>
    anomaly_pub_;
  rclcpp::TimerBase::SharedPtr check_timer_;

  // Prometheusへexportするメトリクス。on_configure()でstart()し、
  // on_cleanup()/デストラクタでstop()する。
  safety_metrics::PrometheusExporter metrics_;
};

}  // namespace heartbeat_monitor

#endif  // HEARTBEAT_MONITOR__HEARTBEAT_MONITOR_NODE_HPP_
