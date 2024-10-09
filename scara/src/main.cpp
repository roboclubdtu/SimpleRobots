#include "sp_scara/node.hpp"
#include <chrono>

namespace simple_robots {

using std::chrono::seconds;

SCARANode::SCARANode() : Simulator("scara") {}

} // namespace simple_robots

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<simple_robots::SCARANode>();
  while (rclcpp::ok())
    rclcpp::spin_some(node);
  rclcpp::shutdown();
  return 0;
}