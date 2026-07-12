#include "estop_bridge/estop_bridge_node.hpp"

#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstring>

using namespace std::chrono_literals;

namespace estop_bridge
{

EstopBridgeNode::EstopBridgeNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("estop_bridge", options),
  kafka_brokers_("hacobot-kafka-kafka-bootstrap.kafka.svc.cluster.local:9092"),
  kafka_topic_("robot-control"),
  kafka_group_id_("estop_bridge"),
  kafka_poll_timeout_ms_(100),
  kafka_host_("hacobot-kafka-kafka-bootstrap.kafka.svc.cluster.local"),
  kafka_port_(9092),
  connect_timeout_ms_(200),
  connectivity_check_period_ms_(2000),
  heartbeat_period_ms_(100),
  assume_healthy_(false),
  is_healthy_(false),
  metrics_port_(9105),
  consumer_running_(false)
{
  this->declare_parameter<std::string>("kafka_brokers", kafka_brokers_);
  this->declare_parameter<std::string>("kafka_topic", kafka_topic_);
  this->declare_parameter<std::string>("kafka_group_id", kafka_group_id_);
  this->declare_parameter<int64_t>("kafka_poll_timeout_ms", kafka_poll_timeout_ms_);

  this->declare_parameter<std::string>("kafka_host", kafka_host_);
  this->declare_parameter<int64_t>("kafka_port", kafka_port_);
  this->declare_parameter<int64_t>("connect_timeout_ms", connect_timeout_ms_);
  this->declare_parameter<int64_t>("connectivity_check_period_ms", connectivity_check_period_ms_);
  this->declare_parameter<int64_t>("heartbeat_period_ms", heartbeat_period_ms_);
  // 暫定バイパス(comm_bridge_heartbeatから引き継ぎ)。開発環境ではDocker Desktopの
  // ネットワーク境界によりKafkaへのTCP到達性チェック自体が常に失敗するため、
  // safety_bringup.launch.pyではtrueで起動している。
  this->declare_parameter<bool>("assume_healthy", assume_healthy_);
  this->declare_parameter<int64_t>("metrics_port", metrics_port_);
}

EstopBridgeNode::~EstopBridgeNode()
{
  stop_consumer_thread();
}

EstopBridgeNode::CallbackReturn EstopBridgeNode::on_configure(
  const rclcpp_lifecycle::State & /*state*/)
{
  kafka_brokers_ = this->get_parameter("kafka_brokers").as_string();
  kafka_topic_ = this->get_parameter("kafka_topic").as_string();
  kafka_group_id_ = this->get_parameter("kafka_group_id").as_string();
  kafka_poll_timeout_ms_ = this->get_parameter("kafka_poll_timeout_ms").as_int();

  kafka_host_ = this->get_parameter("kafka_host").as_string();
  kafka_port_ = this->get_parameter("kafka_port").as_int();
  connect_timeout_ms_ = this->get_parameter("connect_timeout_ms").as_int();
  connectivity_check_period_ms_ = this->get_parameter("connectivity_check_period_ms").as_int();
  heartbeat_period_ms_ = this->get_parameter("heartbeat_period_ms").as_int();
  assume_healthy_ = this->get_parameter("assume_healthy").as_bool();
  is_healthy_ = assume_healthy_;
  metrics_port_ = this->get_parameter("metrics_port").as_int();
  metrics_.start(static_cast<int>(metrics_port_));

  anomaly_pub_ = this->create_publisher<safety_msgs::msg::AnomalyEvent>(
    "/safety/anomaly_event", rclcpp::QoS(10));
  heartbeat_pub_ = this->create_publisher<std_msgs::msg::Empty>(
    "/safety/heartbeat/comm_bridge", rclcpp::QoS(10));

  RCLCPP_INFO(
    this->get_logger(),
    "Configured (kafka_brokers=%s, kafka_topic=%s, assume_healthy=%s)",
    kafka_brokers_.c_str(), kafka_topic_.c_str(), assume_healthy_ ? "true" : "false");

  return CallbackReturn::SUCCESS;
}

EstopBridgeNode::CallbackReturn EstopBridgeNode::on_activate(
  const rclcpp_lifecycle::State & /*state*/)
{
  anomaly_pub_->on_activate();
  heartbeat_pub_->on_activate();

  std::string errstr;
  std::unique_ptr<RdKafka::Conf> conf(RdKafka::Conf::create(RdKafka::Conf::CONF_GLOBAL));
  conf->set("bootstrap.servers", kafka_brokers_, errstr);
  conf->set("group.id", kafka_group_id_, errstr);
  // ブローカー未到達時にconsume()が長時間ブロックしないようにする
  conf->set("socket.timeout.ms", "1000", errstr);

  consumer_.reset(RdKafka::KafkaConsumer::create(conf.get(), errstr));
  if (!consumer_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create Kafka consumer: %s", errstr.c_str());
    return CallbackReturn::FAILURE;
  }

  const RdKafka::ErrorCode subscribe_err = consumer_->subscribe({kafka_topic_});
  if (subscribe_err != RdKafka::ERR_NO_ERROR) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to subscribe to topic '%s': %s",
      kafka_topic_.c_str(), RdKafka::err2str(subscribe_err).c_str());
    return CallbackReturn::FAILURE;
  }

  consumer_running_ = true;
  consumer_thread_ = std::thread(&EstopBridgeNode::kafka_consume_loop, this);

  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(heartbeat_period_ms_),
    std::bind(&EstopBridgeNode::on_heartbeat_timer, this));

  if (!assume_healthy_) {
    connectivity_check_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(connectivity_check_period_ms_),
      std::bind(&EstopBridgeNode::on_connectivity_check_timer, this));
    on_connectivity_check_timer();
  } else {
    // assume_healthy=true(この開発環境の既定)では実際のTCPチェックを行わないため
    // on_connectivity_check_timer()が一度も呼ばれず、ゲージが未設定のままになる。
    // それだとPrometheus/Grafana側でこのノードのメトリクスが常に空に見えてしまうため、
    // バイパス中であることも含めてここで明示的に1度だけ値を設定しておく。
    metrics_.set_gauge(
      "estop_bridge_kafka_reachable", 1.0,
      "1 if the TCP connectivity check to the Kafka broker last succeeded "
      "(always reported as assume_healthy's value when the real check is bypassed)");
  }

  RCLCPP_INFO(
    this->get_logger(), "Activated: consuming '%s' on a dedicated thread", kafka_topic_.c_str());
  return CallbackReturn::SUCCESS;
}

EstopBridgeNode::CallbackReturn EstopBridgeNode::on_deactivate(
  const rclcpp_lifecycle::State & /*state*/)
{
  heartbeat_timer_.reset();
  connectivity_check_timer_.reset();
  stop_consumer_thread();
  anomaly_pub_->on_deactivate();
  heartbeat_pub_->on_deactivate();

  RCLCPP_INFO(this->get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

EstopBridgeNode::CallbackReturn EstopBridgeNode::on_cleanup(
  const rclcpp_lifecycle::State & /*state*/)
{
  anomaly_pub_.reset();
  heartbeat_pub_.reset();
  metrics_.stop();

  RCLCPP_INFO(this->get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

EstopBridgeNode::CallbackReturn EstopBridgeNode::on_shutdown(
  const rclcpp_lifecycle::State & /*state*/)
{
  heartbeat_timer_.reset();
  connectivity_check_timer_.reset();
  stop_consumer_thread();
  anomaly_pub_.reset();
  heartbeat_pub_.reset();
  metrics_.stop();

  RCLCPP_INFO(this->get_logger(), "Shutdown");
  return CallbackReturn::SUCCESS;
}

void EstopBridgeNode::stop_consumer_thread()
{
  consumer_running_ = false;
  if (consumer_thread_.joinable()) {
    consumer_thread_.join();
  }
  if (consumer_) {
    consumer_->close();
    consumer_.reset();
  }
}

bool EstopBridgeNode::is_estop_command(const std::string & payload)
{
  std::string trimmed = payload;
  trimmed.erase(
    trimmed.begin(),
    std::find_if(
      trimmed.begin(), trimmed.end(),
      [](unsigned char ch) {return !std::isspace(ch);}));
  trimmed.erase(
    std::find_if(
      trimmed.rbegin(), trimmed.rend(),
      [](unsigned char ch) {return !std::isspace(ch);}).base(),
    trimmed.end());
  return trimmed == "ESTOP";
}

void EstopBridgeNode::kafka_consume_loop()
{
  // このループは専用スレッド上で動く。ROS executorのコールバック(Nav2/センサー等)の
  // 負荷とは無関係に、Kafkaからのメッセージを低遅延で処理し続けることが目的。
  while (consumer_running_) {
    std::unique_ptr<RdKafka::Message> msg(consumer_->consume(kafka_poll_timeout_ms_));

    switch (msg->err()) {
      case RdKafka::ERR_NO_ERROR:
        {
          const std::string payload(
            static_cast<const char *>(msg->payload()), msg->len());
          if (is_estop_command(payload)) {
            safety_msgs::msg::AnomalyEvent event;
            event.stamp = this->now();
            event.source = "estop_bridge";
            event.severity = safety_msgs::msg::AnomalyEvent::CRITICAL;
            event.reason = "remote E-Stop command received via Kafka topic '" +
              kafka_topic_ + "'";

            // publish()はKafkaコンシューマ専用スレッドから直接呼んでいる
            // (rclcppのPublisher::publish()は複数スレッドから呼んでも安全)。
            // ROS executorのスレッドを経由させないことで、E-Stop通知の遅延を
            // 他ノードの負荷から切り離している。
            if (anomaly_pub_->is_activated()) {
              anomaly_pub_->publish(event);
            }
            metrics_.increment_counter(
              "estop_bridge_estop_received_total",
              "Number of remote E-Stop commands received via Kafka");
            RCLCPP_ERROR(this->get_logger(), "E-Stop command received: %s", payload.c_str());
          }
          break;
        }
      case RdKafka::ERR__TIMED_OUT:
      case RdKafka::ERR__PARTITION_EOF:
        break;
      default:
        RCLCPP_DEBUG(
          this->get_logger(), "Kafka consume error: %s", msg->errstr().c_str());
        break;
    }
  }
}

void EstopBridgeNode::on_heartbeat_timer()
{
  if (!is_healthy_) {
    return;
  }
  if (heartbeat_pub_->is_activated()) {
    heartbeat_pub_->publish(std_msgs::msg::Empty());
  }
}

void EstopBridgeNode::on_connectivity_check_timer()
{
  const bool reachable = check_tcp_reachable(kafka_host_, kafka_port_, connect_timeout_ms_);
  if (reachable != is_healthy_) {
    RCLCPP_WARN(
      this->get_logger(), "Kafka connectivity (%s:%ld) changed: %s -> %s",
      kafka_host_.c_str(), kafka_port_,
      is_healthy_ ? "reachable" : "unreachable",
      reachable ? "reachable" : "unreachable");
  }
  is_healthy_ = reachable;
  metrics_.set_gauge(
    "estop_bridge_kafka_reachable", reachable ? 1.0 : 0.0,
    "1 if the TCP connectivity check to the Kafka broker last succeeded "
    "(always reported as assume_healthy's value when the real check is bypassed)");
}

bool EstopBridgeNode::check_tcp_reachable(
  const std::string & host, int port, int timeout_ms)
{
  struct addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  struct addrinfo * res = nullptr;
  const std::string port_str = std::to_string(port);
  if (getaddrinfo(host.c_str(), port_str.c_str(), &hints, &res) != 0 || res == nullptr) {
    return false;
  }

  bool connected = false;
  for (struct addrinfo * rp = res; rp != nullptr; rp = rp->ai_next) {
    const int fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
    if (fd < 0) {
      continue;
    }

    const int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    const int rc = connect(fd, rp->ai_addr, rp->ai_addrlen);
    if (rc == 0) {
      connected = true;
    } else if (errno == EINPROGRESS) {
      struct pollfd pfd{};
      pfd.fd = fd;
      pfd.events = POLLOUT;
      const int poll_rc = poll(&pfd, 1, timeout_ms);
      if (poll_rc > 0 && (pfd.revents & POLLOUT)) {
        int so_error = 0;
        socklen_t len = sizeof(so_error);
        if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_error, &len) == 0 && so_error == 0) {
          connected = true;
        }
      }
    }

    close(fd);
    if (connected) {
      break;
    }
  }

  freeaddrinfo(res);
  return connected;
}

}  // namespace estop_bridge
