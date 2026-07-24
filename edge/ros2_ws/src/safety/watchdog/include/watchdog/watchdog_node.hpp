#ifndef WATCHDOG__WATCHDOG_NODE_HPP_
#define WATCHDOG__WATCHDOG_NODE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "safety_metrics/prometheus_exporter.hpp"
#include "safety_msgs/msg/anomaly_event.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"

namespace watchdog
{

// カメラ・LiDAR等、実際のセンサートピックの更新間隔を直接監視するlifecycle node。
// heartbeat_monitorと違い、監視対象は既存のセンサーメッセージ型
// (sensor_msgs/LaserScan, sensor_msgs/Image 等)であり型が topic ごとに異なるため、
// メッセージ内容を意識しない rclcpp::GenericSubscription で「届いたかどうか」だけを見る。
// 一定時間(デフォルト500ms)更新が無ければ、/safety/anomaly_event に
// severity="CRITICAL"(DEGRADEDを経由せず即SAFE_STOP)を通知する。
//
// on_activate()直後はセンサー側もまだ起動中で未publishなことが普通にあるため、
// startup_grace_period_ms(既定値はtimeout_msより大きい)の間だけ通常のtimeout_msより
// 緩い閾値を適用し、システム起動直後の誤検知を防ぐ。
//
// /safety/recovery_commandを受信すると、is_stale_フラグを全監視対象についてリセットする。
// safety_state_machineがNORMALへ復旧しても実際にはセンサーが復旧していない場合に、
// 次のcheck_timeouts()周期で即座に再検知・再通知できるようにするため
// (heartbeat_monitorと同じ設計、詳細はそちらのコメント参照)。
//
// /safety/sensors_ok(transient_local)に、監視対象が全て非stale状態かどうかを
// 継続的にpublishする。/safety/anomaly_eventは途絶の"検知イベント"のみでSAFE_STOP後は
// 沈黙するため、safety_state_machineがSAFE_STOP中に「今センサーが実際に生きているか」を
// リアルタイムに判定する材料がこの状態だけ別に必要(センサー正常時はSAFE_STOP中でも
// cmd_velを中継してよい、という設計のため)。
class WatchdogNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit WatchdogNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  struct MonitoredTopic
  {
    std::string name;
    std::string type;
  };

  void check_timeouts();
  void on_recovery_command(const std_msgs::msg::Empty::SharedPtr msg);
  void update_sensors_ok();
  static std::vector<MonitoredTopic> parse_monitored_topics(
    const std::vector<std::string> & raw_entries);

  std::vector<MonitoredTopic> monitored_topics_;
  int64_t timeout_ms_;
  int64_t check_period_ms_;
  int64_t startup_grace_period_ms_;
  int64_t metrics_port_;

  rclcpp::Time activated_at_;
  std::map<std::string, rclcpp::Time> last_received_;
  std::map<std::string, bool> is_stale_;
  bool sensors_ok_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> subs_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr recovery_sub_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<safety_msgs::msg::AnomalyEvent>>
    anomaly_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>> sensors_ok_pub_;
  rclcpp::TimerBase::SharedPtr check_timer_;

  safety_metrics::PrometheusExporter metrics_;
};

}  // namespace watchdog

#endif  // WATCHDOG__WATCHDOG_NODE_HPP_
