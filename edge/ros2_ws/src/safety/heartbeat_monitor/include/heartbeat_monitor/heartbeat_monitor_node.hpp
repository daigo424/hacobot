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

// 監視対象ノード(Nav2, 通信ブリッジ)が /safety/heartbeat/<source> へpublishする
// ハートビートの生存を、DDS QoSのLiveliness(ManualByTopic)を使って監視し、
// 途絶したら /safety/anomaly_event に異常イベントをpublishするlifecycle node。
//
// 生存判定自体はDDSミドルウェアに委譲する(publisher側もliveliness_lease_msを揃えた
// 同じQoSでpublishしており、publish()自体が生存主張を兼ねる)。このノードは
// Subscriptionのliveliness_callback(QOSLivelinessChangedInfo)を購読するだけで、
// 自前のポーリングタイマーで経過時間を計算することはしない。
//
// on_activate()直後は監視対象側の購読がまだDDS上でマッチしておらず、一時的に
// alive_count==0に見えることがあるため、startup_grace_period_msの間だけ
// liveliness_callbackでの非生存判定を無視し、システム起動直後の誤検知を防ぐ。
//
// liveliness_callbackは「alive⇔not_aliveの変化」でしか発火しないため、
// 一度も生存を主張しないまま(=publisherが起動すらしていない)source は
// コールバックが一切発火せず検知できない。これを防ぐため、
// startup_grace_period_ms経過時点で一度だけ「一度でも生存を確認できたか」
// (ever_alive_)をチェックし、確認できていなければ異常として通知する。
//
// /safety/recovery_commandを受信すると、is_stale_フラグ(異常を既に通知済みかどうか)を
// 全監視対象についてリセットする。ただしポーリングタイマーが無いため、リセット直後に
// 「まだ実際には非生存のまま」のsourceがあれば、この場で即座に再度異常をpublishする
// (currently_alive_に基づく。新しいハートビートが届いた場合のみ正常とみなす、
// フェイルセーフ優先の設計を踏襲する)。
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
  void on_liveliness_changed(
    const std::string & source, rclcpp::QOSLivelinessChangedInfo & info);
  void on_recovery_command(const std_msgs::msg::Empty::SharedPtr msg);
  void publish_anomaly(const std::string & source);
  void check_startup_liveliness();
  bool in_startup_grace() const;

  std::vector<std::string> monitored_sources_;
  int64_t liveliness_lease_ms_;
  int64_t startup_grace_period_ms_;
  int64_t metrics_port_;

  rclcpp::Time activated_at_;
  std::map<std::string, bool> currently_alive_;
  std::map<std::string, bool> ever_alive_;
  std::map<std::string, bool> is_stale_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr> heartbeat_subs_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr recovery_sub_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<safety_msgs::msg::AnomalyEvent>>
    anomaly_pub_;
  rclcpp::TimerBase::SharedPtr startup_check_timer_;

  // Prometheusへexportするメトリクス。on_configure()でstart()し、
  // on_cleanup()/デストラクタでstop()する。
  safety_metrics::PrometheusExporter metrics_;
};

}  // namespace heartbeat_monitor

#endif  // HEARTBEAT_MONITOR__HEARTBEAT_MONITOR_NODE_HPP_
