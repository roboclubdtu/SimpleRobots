#include "simple_robots/kinematic_simulator.hpp"

namespace simple_robots {

KinematicSimulator::KinematicSimulator(const std::string &name)
    : rclcpp::Node(name, rclcpp::NodeOptions()) {

  robot_urdf = declare_parameter(ParamName::ROBOT_DESC, Default::ROBOT_DESC);
  state_publisher = create_publisher<JointState>(Topics::JOINT_STATE, QOS);
  loop_duration = std::chrono::milliseconds(
      1000 / declare_parameter(ParamName::LOOP_HZ, Default::LOOP_HZ));

  if (robot_urdf.empty())
    throw std::runtime_error(Errors::EMPTY_URDF);
  auto loader = URDFLoader(robot_urdf, get_logger());
  joint_state = loader.getExtendedJointState();

  // Start simulator
  main_loop = create_wall_timer(loop_duration,
                                std::bind(&KinematicSimulator::loop, this));
}

void KinematicSimulator::loop() {
  joint_state.state.header.stamp = get_clock()->now();
  state_publisher->publish(joint_state.state);
}

} // namespace simple_robots