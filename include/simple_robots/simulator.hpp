#ifndef SIMPLE_ROBOT_KINEMATIC_SIMU
#define SIMPLE_ROBOT_KINEMATIC_SIMU

#include <rclcpp/rclcpp.hpp>

#include "simple_robots/urdf_loader.hpp"

namespace simple_robots {

using sensor_msgs::msg::JointState;
using namespace rclcpp;

/**
 * @brief General kinematic simulator for robot based on URDF description
 *
 */
class Simulator : public Node {
 public:
  struct Topics {
    constchar JOINT_STATE{"joint_states"};  //< Topic for exporting the simulated robot joint states
    constchar CMD_VAL{"cmd_vel"};           //< Topic for input velocities command
  };
  struct Param {
    constchar ROBOT_DESC{"robot_description"};        //< URDF description of the robot
    constchar LOOP_HZ{"loop_hz"};                     //< Main loop frequency
    constchar M_ANG_VEL{"default_max_ang_velocity"};  //< Maximum angle velocity (rad/s) if URDF value is 0
    constchar M_ANG_EFF{"default_max_ang_efforts"};   //< Maximum angle effort (Nm) if URDF value is 0
    constchar M_LIN_VEL{"default_max_lin_velocity"};  //< Maximum linear velocity (m/s) if URDF value is 0
    constchar M_LIN_EFF{"default_max_lin_efforts"};   //< Maximum linear effort (N) if URDF value is 0
  };
  struct Default {
    constchar ROBOT_DESC{""};
    constval int LOOP_HZ{20};
    constval double M_ANG_VEL{0.5};  //< Maximum angle velocity (rad/s) if URDF value is 0
    constval double M_LIN_VEL{0.2};  //< Maximum linear velocity (m/s) if URDF value is 0
    constval double M_ANG_EFF{0.1};  //< Maximum angle effort (Nm) if URDF value is 0
    constval double M_LIN_EFF{1};    //< Maximum linear effort (N) if URDF value is 0
  };
  typedef std::shared_ptr<Simulator> SharedPtr;

  Simulator(const std::string &name);

 private:
  /**
   * @brief Main loop of the simulator
   */
  void loop();

  /**
   * @brief Compute the new state with a velocity goal for the joints.
   */
  void compute_with_velocity_cmd(double dt);

  /**
   * @brief Callback function for input velocity goals
   */
  void clbk_vel_cmd(const JointState::SharedPtr);

 private:
  struct Errors {
    constchar EMPTY_URDF{"The given robot URDF XML string is empty!"};
    constchar UNIMPLEMENT_EFFORTS{"Effort commands not implemented!"};
    constchar UNSUPPORTED_JOINT_TYPE{"The joint type %s for joint %s is unsupported!"};
    constchar UNKNOWN_JOINT{"Using unknown joint %s"};
  };
  constval float THROTTLE_DUR_MS{0.5};
  constval int QOS{10};

  // Internal computation components
  data::ExtendedJointState joint_state;     //< Internal joint state storage
  std::string robot_urdf;                   //< String representation of the XML URDF robot
  std::chrono::milliseconds loop_duration;  //< The duration of a loop of the simulator
  bool first_iter = true;

  data::Limits lim_ang_vel, lim_ang_eff, lim_lin_vel, lim_lin_eff;

  // ROS components
  Publisher<sensor_msgs::msg::JointState>::SharedPtr state_publisher;
  Subscription<JointState>::SharedPtr vel_cmd_subscriber;
  TimerBase::SharedPtr main_loop;
};

}  // namespace simple_robots

#endif