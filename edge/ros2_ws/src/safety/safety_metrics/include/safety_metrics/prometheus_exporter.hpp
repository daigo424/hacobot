#ifndef SAFETY_METRICS__PROMETHEUS_EXPORTER_HPP_
#define SAFETY_METRICS__PROMETHEUS_EXPORTER_HPP_

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

namespace safety_metrics
{

// 各安全ノード(heartbeat_monitor, safety_state_machine, watchdog,
// nav2_heartbeat_adapter, estop_bridge)がPrometheusへメトリクスをexportするための
// 最小限のヘッダオンリー実装。prometheus-cpp等の外部ライブラリを追加導入する代わりに、
// 「HTTP GETが来たら現在のメトリクスをPrometheusテキスト形式で返すだけ」の
// 生ソケットサーバーを自前で書いている(POSIXソケットAPIはestop_bridgeのTCP到達性
// チェックで既に使っており、その延長線上の実装)。
//
// 使い方の例(on_activate内):
//   metrics_.start(9101);
//   ...
//   metrics_.increment_counter("heartbeat_monitor_anomaly_total{source=\"nav2\"}",
//                               "Number of heartbeat timeout anomalies detected");
//
// 注意: フル機能のHTTPサーバーではない(リクエストの中身は読み捨て、パスによる
// 出し分けもしない)。GET / でもGET /metrics でも常に同じメトリクス全量を返す。
// Prometheusのスクレイピングにはこれで十分だが、汎用HTTPクライアントとしての
// 互換性は保証しない。
class PrometheusExporter
{
public:
  PrometheusExporter() = default;

  ~PrometheusExporter()
  {
    stop();
  }

  void start(int port)
  {
    port_ = port;
    running_ = true;
    server_thread_ = std::thread(&PrometheusExporter::serve_loop, this);
  }

  void stop()
  {
    running_ = false;
    if (listen_fd_ >= 0) {
      shutdown(listen_fd_, SHUT_RDWR);
      close(listen_fd_);
      listen_fd_ = -1;
    }
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
  }

  void set_gauge(const std::string & metric, double value, const std::string & help = "")
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gauges_[metric] = value;
    if (!help.empty()) {
      help_[base_name(metric)] = help;
    }
  }

  void increment_counter(const std::string & metric, const std::string & help = "")
  {
    std::lock_guard<std::mutex> lock(mutex_);
    counters_[metric] += 1.0;
    if (!help.empty()) {
      help_[base_name(metric)] = help;
    }
  }

private:
  // "foo_total{label=\"x\"}" -> "foo_total" (HELP/TYPE行はラベル無しの基本名単位で出す)
  static std::string base_name(const std::string & metric)
  {
    const auto brace = metric.find('{');
    return brace == std::string::npos ? metric : metric.substr(0, brace);
  }

  void serve_loop()
  {
    listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
      return;
    }
    const int opt = 1;
    setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<uint16_t>(port_));
    if (bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      close(listen_fd_);
      listen_fd_ = -1;
      return;
    }
    if (listen(listen_fd_, 4) < 0) {
      close(listen_fd_);
      listen_fd_ = -1;
      return;
    }

    while (running_) {
      sockaddr_in client_addr{};
      socklen_t addr_len = sizeof(client_addr);
      const int client_fd = accept(
        listen_fd_, reinterpret_cast<sockaddr *>(&client_addr), &addr_len);
      if (client_fd < 0) {
        continue;  // running_==falseならstop()側でlisten_fdを閉じてループを抜ける
      }

      char buf[1024];
      recv(client_fd, buf, sizeof(buf) - 1, 0);  // リクエスト内容は使わず読み捨てるだけ

      const std::string body = render();
      std::ostringstream response;
      response << "HTTP/1.1 200 OK\r\n"
               << "Content-Type: text/plain; version=0.0.4\r\n"
               << "Content-Length: " << body.size() << "\r\n"
               << "Connection: close\r\n\r\n"
               << body;
      const std::string response_str = response.str();
      send(client_fd, response_str.c_str(), response_str.size(), 0);
      close(client_fd);
    }
  }

  std::string render()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::ostringstream out;
    for (const auto & kv : help_) {
      out << "# HELP " << kv.first << " " << kv.second << "\n";
    }
    for (const auto & kv : gauges_) {
      out << "# TYPE " << base_name(kv.first) << " gauge\n"
          << kv.first << " " << kv.second << "\n";
    }
    for (const auto & kv : counters_) {
      out << "# TYPE " << base_name(kv.first) << " counter\n"
          << kv.first << " " << kv.second << "\n";
    }
    return out.str();
  }

  int port_ = 0;
  int listen_fd_ = -1;
  std::atomic<bool> running_{false};
  std::thread server_thread_;

  std::mutex mutex_;
  std::map<std::string, double> gauges_;
  std::map<std::string, double> counters_;
  std::map<std::string, std::string> help_;
};

}  // namespace safety_metrics

#endif  // SAFETY_METRICS__PROMETHEUS_EXPORTER_HPP_
