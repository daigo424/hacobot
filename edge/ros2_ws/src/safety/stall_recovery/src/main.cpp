#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "stall_recovery/stall_recovery_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<stall_recovery::StallRecoveryNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
