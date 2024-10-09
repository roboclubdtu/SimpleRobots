#include "simple_robots/simulator.hpp"

namespace simple_robots {

using namespace std::placeholders;

Simulator::Simulator(const std::string &name) : rclcpp::Node(name, rclcpp::NodeOptions()) {
  // Parse parameters
  robot_urdf = declare_parameter(Param::ROBOT_DESC, Default::ROBOT_DESC);
  loop_duration = std::chrono::milliseconds(1000 / declare_parameter(Param::LOOP_HZ, Default::LOOP_HZ));

  // Maximum vel and efforts for joints
  lim_ang_vel = data::Limits(declare_parameter(Param::M_ANG_VEL, Default::M_ANG_VEL));
  lim_ang_eff = data::Limits(declare_parameter(Param::M_ANG_EFF, Default::M_ANG_EFF));
  lim_lin_vel = data::Limits(declare_parameter(Param::M_LIN_VEL, Default::M_LIN_VEL));
  lim_lin_eff = data::Limits(declare_parameter(Param::M_LIN_EFF, Default::M_LIN_EFF));

  // Load model
  if (robot_urdf.empty()) throw std::runtime_error(Errors::EMPTY_URDF);
  auto loader = URDFLoader(robot_urdf, get_logger());
  joint_state = loader.getExtendedJointState();

  // ROS IO
  state_publisher = create_publisher<JointState>(Topics::JOINT_STATE, QOS);
  vel_cmd_subscriber = create_subscription<JointState>(Topics::CMD_VAL, QOS, std::bind(&Simulator::clbk_vel_cmd, this, _1));

  // // Start simulator
  main_loop = create_wall_timer(loop_duration, std::bind(&Simulator::loop, this));
}

///////////////////////////////////////////////////////////////////////////////
// Callback functions
///////////////////////////////////////////////////////////////////////////////

void Simulator::clbk_vel_cmd(const JointState::SharedPtr js) {
  // Clearing old goal
  joint_state.goal = data::GoalType::VELOCITY;
  for (ulong joint = 0; joint < joint_state.n_joints; joint++) joint_state.goals[joint] = 0.0;
  for (ulong joint = 0; joint < js->name.size(); joint++) {
    // Get joint index from the name
    if (auto it = joint_state.name_map.find(js->name[joint]); it == joint_state.name_map.end()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), THROTTLE_DUR_MS, Errors::UNKNOWN_JOINT, js->name[joint].c_str());
      continue;
    } else {
      joint_state.goals[it->second] = js->velocity[joint];
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
// Simulation loop
///////////////////////////////////////////////////////////////////////////////

void Simulator::loop() {
  static rclcpp::Time last_time = get_clock()->now();
  if (first_iter) {
    first_iter = false;
    return;
  }
  auto dt = (last_time - get_clock()->now()).seconds();
  last_time = get_clock()->now();

  // Compute the output
  if (joint_state.goal == data::GoalType::VELOCITY) compute_with_velocity_cmd(dt);

  // Export it
  joint_state.state.header.stamp = get_clock()->now();
  state_publisher->publish(joint_state.state);
}

void Simulator::compute_with_velocity_cmd(double dt) {
  // Iterate over the joints
  double dv = 0;
  double old_p = 0;
  double new_p = 0;

  auto &joint_vel = joint_state.state.velocity;
  auto &joint_pos = joint_state.state.position;
  for (ulong joint = 0; joint < joint_state.n_joints; joint++) {
    auto &config = joint_state.configs[joint];

    // Respect velocity limits ?
    dv = joint_state.goals[joint] - joint_state.state.velocity[joint];
    joint_vel[joint] += dv;

    if (config.lim_vel.max == 0) {
      switch (config.type) {
        case data::JointType::REVOLUTE:
          joint_vel[joint] = lim_ang_vel.saturate(joint_vel[joint]);
          break;
        case data::JointType::PRISMATIC:
          joint_vel[joint] = lim_lin_vel.saturate(joint_vel[joint]);
          break;
        default:
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), THROTTLE_DUR_MS, Errors::UNSUPPORTED_JOINT_TYPE,
                               data::JointType::getJointTypeName(config.type), config.name.c_str());
      }
    } else {
      joint_vel[joint] = config.lim_vel.saturate(joint_vel[joint]);
    }

    // Compute new pos
    old_p = joint_pos[joint];
    new_p = old_p + joint_vel[joint] * dt;
    auto &pos_lim = config.lim_pos;
    if (!pos_lim.inside(new_p)) {
      new_p = pos_lim.saturate(new_p);
      joint_vel[joint] = (new_p - old_p) / dt;
    }
    joint_pos[joint] = new_p;
  }
}

}  // namespace simple_robots