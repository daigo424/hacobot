#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "safety_state_machine/safety_state_machine_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<safety_state_machine::SafetyStateMachineNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
