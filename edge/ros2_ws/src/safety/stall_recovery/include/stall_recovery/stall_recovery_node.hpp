#ifndef STALL_RECOVERY__STALL_RECOVERY_NODE_HPP_
#define STALL_RECOVERY__STALL_RECOVERY_NODE_HPP_

#include <deque>
#include <memory>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "safety_metrics/prometheus_exporter.hpp"
#include "safety_msgs/msg/anomaly_event.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "slam_toolbox/srv/pause.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

namespace stall_recovery
{

// 壁等にホイールを押し付けられて空転すると、オドメトリだけが実際の動きなく前進し続け、
// slam_toolboxの局所スキャンマッチングでは(平らな壁沿いの並進は区別する手がかりが乏しく)
// この暴走を訂正しきれず地図が壊れる。この現象はホイールエンコーダに起因するため、
// エンコーダを経由しないLiDAR生スキャンで「指令通り動いているはずなのに、実際には
// 動いていない」ことを直接検知する。判定は2種類を併用する:
// (1) スキャン全体の類似度(平らな壁沿いのスリップに強いが、円柱等の曲面障害物では
//     わずかな回転でも見え方が大きく変わり、類似度が下がって検知漏れすることがある)
// (2) 最小距離(直近の障害物までの距離)がほぼ一定のまま張り付いていないか
//     (障害物の形状に依存しないため、円柱のような曲面に押し付けられている場合も拾える)
// いずれかが成立すればスタックとみなし、safety_state_machineの人手復旧チェーン
// (SAFE_STOP->MANUAL_RECOVERY)を経由せず自分で離脱動作をしてから探索へ制御を戻す。
// 離脱動作は「少し後退→その場旋回」の2段階。直進後退だけだと壁の角に挟まった場合に
// 別の壁へ押し付けられるだけで抜け出せないため旋回を行うが、旋回だけだと接触点の
// 直近で機体の別の部分(角の反対側等)が新たに接触することがあるため、旋回前に
// ごく短時間・低速で後退し隙間を作ってから旋回する。
// クールダウン終了後、直前の離脱動作からconsecutive_stall_gap_sec_以内に
// 通常の検知ロジックが再びスタックを確認した場合は、既に後退で隙間を作った直後
// とみなし後退を省略してそのまま旋回する(毎回後退すると、旋回だけでは向きを
// 変えきれない障害物の前でいつまでも後退と旋回を繰り返しかねないため)。
// 「近くに何かある」という距離だけの判定は、離脱済みでも別の物に近いだけの
// 状況と区別できず誤検知するため使わない。必ず「指令通り動いているのに
// スキャンが変化しない」という元の堅牢な判定を経由させる。
//
// safety_state_machineがSAFE_STOP/MANUAL_RECOVERY中はcmd_velを強制的にゼロに
// しており、その間はNav2が指令(/cmd_vel_nav2_raw)を出し続けていてもロボットは
// 実際には全く動かない。これをホイール空転と誤検知しないよう、/safety/stateが
// NORMAL/DEGRADED(実際にcmd_velが中継される状態)の間だけスタック検知を行う。
//
// Nav2の生cmd_vel(/cmd_vel_nav2_raw)を中継してsafety_state_machineが読む
// /cmd_vel_nav2へ流す中間ノードとして動作する(通常時は素通し、スタック検知時のみ
// 旋回指令に差し替える)。既存のWARNING/CRITICAL経路とは独立した第三の反応だが、
// 旋回を繰り返しても解消しない場合は人間の判断が必要と判断し、CRITICALとして
// /safety/anomaly_eventへ通知しsafety_state_machine側のSAFE_STOP/MANUAL_RECOVERYへ
// エスカレーションする。
//
// スタックを検知してから旋回で実際に離脱するまでの間も、ホイール空転由来の
// 誤ったスキャンがslam_toolboxのpose graphに取り込まれ地図を壊し続けてしまうため、
// スタック検知の瞬間にslam_toolboxのpause_new_measurementsサービスで地図生成自体を
// 一時停止し、PASSTHROUGHへ復帰する時点で再開する。
//
// cmd_velを横取りするだけではNav2/explore_lite側は介入に気づかず、Nav2は
// 元の計画通りの指令を出し続け、Nav2自身のprogress_checkerとも無関係に
// 二重でリカバリーが発火しうる。スタック検知時にexplore_liteのexplore/resumeへ
// falseを送り(explore_lite側でNav2の現在のゴールをキャンセルさせ、新規ゴール
// 送信も止める)、復帰時にtrueを送ることで、explore_lite/Nav2側の制御と
// 明示的に排他させる。
class StallRecoveryNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit StallRecoveryNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  enum class Mode
  {
    PASSTHROUGH,
    BACKING_OFF,
    TURNING,
    COOLDOWN,
    ESCALATED,
  };

  // テスト用に現在の内部モードを直接参照できるようにする
  Mode current_mode() const {return mode_;}

  static std::string to_string(Mode mode);

private:
  void publish_mode();
  void publish_explore_resume(bool resume);
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void on_cmd_vel_raw(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void on_recovery_command(const std_msgs::msg::Empty::SharedPtr msg);
  void on_safety_state(const std_msgs::msg::String::SharedPtr msg);
  void on_control_timer();
  bool is_commanding_motion() const;
  double compute_similarity(const sensor_msgs::msg::LaserScan & a, const sensor_msgs::msg::LaserScan & b) const;
  static double compute_min_range(const sensor_msgs::msg::LaserScan & s);
  void enter_backing_off();
  void enter_turning();
  void enter_cooldown();
  void enter_passthrough();
  void escalate(const std::string & reason);
  void set_slam_paused(bool paused);
  // recent_stall_times_に今回の発生を記録し、繰り返し回数がrepeated_stall_limit_を
  // 超えていればescalate()を呼んでtrueを返す(呼び出し側はtrueならその場で処理を止める)
  bool record_stall_and_maybe_escalate();

  // パラメータ
  double compare_interval_sec_;
  double similarity_threshold_;
  double range_epsilon_m_;
  double near_obstacle_threshold_m_;
  double stall_confirm_sec_;
  double backoff_duration_sec_;
  double backoff_linear_x_;
  double turn_duration_sec_;
  double turn_angular_z_;
  double cooldown_sec_;
  double consecutive_stall_gap_sec_;
  double min_linear_cmd_;
  double min_angular_cmd_;
  double cmd_freshness_sec_;
  int64_t repeated_stall_window_sec_;
  int64_t repeated_stall_limit_;
  int64_t control_period_ms_;
  int64_t metrics_port_;

  Mode mode_;
  sensor_msgs::msg::LaserScan::SharedPtr reference_scan_;
  rclcpp::Time reference_scan_time_;
  geometry_msgs::msg::TwistStamped::SharedPtr latest_cmd_;
  rclcpp::Time latest_cmd_time_;
  rclcpp::Time stall_since_;
  bool stall_timer_running_;
  rclcpp::Time mode_change_time_;
  std::deque<rclcpp::Time> recent_stall_times_;
  // このノード自身がslam_toolboxを一時停止させたかどうか(ESCALATED経由での
  // enter_passthrough()では一時停止していないため、そこでは再開呼び出しをしない)
  bool slam_paused_;
  // 直前にCOOLDOWNからPASSTHROUGHへ正常に戻った時刻(後退を省略すべきか判定する基準。
  // ESCALATED経由の復旧では設定しない)
  std::optional<rclcpp::Time> last_recovery_end_time_;
  // safety_state_machineの現在の状態。NORMAL/DEGRADED以外はcmd_velが強制的に
  // ゼロにされておりスタック検知の根拠にならないため、これらの間のみ検知する
  std::string safety_state_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_raw_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr recovery_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr safety_state_sub_;
  rclcpp::Client<slam_toolbox::srv::Pause>::SharedPtr pause_client_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>> cmd_vel_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<safety_msgs::msg::AnomalyEvent>> anomaly_pub_;
  // RViz等での可視化用に現在モードを文字列でpublishする(制御ロジックには使わない)
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> mode_pub_;
  // スタック検知時にexplore_liteへ一時停止(false)/再開(true)を明示的に伝える。
  // explore_lite側はfalseを受けるとNav2の現在のゴールをキャンセルしてくれる
  // (CANCELEDはフロンティアをブラックリストしない)ため、Nav2自身が「横取り」に
  // 気づかず指令を出し続けたり、Nav2自身のprogress_checkerと二重にリカバリーが
  // 発火したりする競合を避けられる。
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>> explore_resume_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  safety_metrics::PrometheusExporter metrics_;
};

}  // namespace stall_recovery

#endif  // STALL_RECOVERY__STALL_RECOVERY_NODE_HPP_
