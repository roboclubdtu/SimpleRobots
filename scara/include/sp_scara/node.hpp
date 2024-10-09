#ifndef SIMPLE_ROBOT_SCARA_NODE
#define SIMPLE_ROBOT_SCARA_NODE

#include "simple_robots/simulator.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>

namespace simple_robots {

using namespace rclcpp;
using sensor_msgs::msg::JointState;

class SCARANode : public Simulator {
public:
  SCARANode();
};

} // namespace simple_robots

#endif