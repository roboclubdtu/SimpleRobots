#ifndef SIMPLE_ROBOT_KINEMATIC_SIMU
#define SIMPLE_ROBOT_KINEMATIC_SIMU

#include "simple_robots/urdf_loader.hpp"
#include <rclcpp/rclcpp.hpp>

namespace simple_robots {

using sensor_msgs::msg::JointState;
using namespace rclcpp;

/**
 * @brief General kinematic simulator for robot based on URDF description
 *
 */
class KinematicSimulator : public Node {
public:
#define constchar static constexpr const char *
  struct Topics {
    constchar JOINT_STATE{"joint_states"};
  };
  struct ParamName {
    constchar ROBOT_DESC{"robot_dsescription"};
    constchar LOOP_HZ{"loop_hz"};
  };
  struct Default {
    constchar ROBOT_DESC{""};
    static constexpr int LOOP_HZ{20};
  };
  typedef std::shared_ptr<KinematicSimulator> SharedPtr;

  KinematicSimulator(const std::string &name);

private:
  void loop();

private:
  struct Errors {
    constchar EMPTY_URDF{"The given robot URDF XML string is empty!"};
  };
  static constexpr int QOS = 10;

  // Internal computation components
  data::ExtendedJointState joint_state;
  std::string robot_urdf;
  std::chrono::milliseconds loop_duration;

  // ROS components
  Publisher<sensor_msgs::msg::JointState>::SharedPtr state_publisher;
  TimerBase::SharedPtr main_loop;
};

} // namespace simple_robots

#endif