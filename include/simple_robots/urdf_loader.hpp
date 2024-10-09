#ifndef SIMPLE_ROBOT_URDF
#define SIMPLE_ROBOT_URDF

#include <urdf/model.h>

#include <iostream>
#include <rclcpp/logging.hpp>
#include <string>

#include "simple_robots/data_struct.hpp"

namespace simple_robots {

class URDFLoader {
 public:
  typedef std::shared_ptr<URDFLoader> SharedPtr;

  URDFLoader() {}
  URDFLoader(const std::string &urdf_xml, const rclcpp::Logger &logger) : urdf_xml(urdf_xml) {
    if (urdf_xml.empty()) throw std::runtime_error("The URDF xml shouldn't be empty.");
    extract_extended_joint_state(logger);
  }

  static SharedPtr make(const std::string &urdf_xml, const rclcpp::Logger &logger) { return std::make_shared<URDFLoader>(urdf_xml, logger); }

  data::ExtendedJointState getExtendedJointState() const { return joint_state; };

 private:
  void extract_extended_joint_state(const rclcpp::Logger &logger) {
    if (!model.initString(urdf_xml)) throw std::runtime_error("Couldn't initialize URDF model from XML string");

    // Parse joints
    for (auto const &[name, joint] : model.joints_) {
      auto type = joint->type;

      RCLCPP_INFO(logger, "Found joint %s of type %s", name.c_str(), data::JointType::getJointTypeName(joint));
      if (!(type == urdf::Joint::REVOLUTE || type == urdf::Joint::PRISMATIC)) continue;

      // Import it in the extended joint state member
      joint_state.name_map.insert_or_assign(name, joint_state.configs.size());
      joint_state.configs.push_back(data::JointConfig(joint));

      auto config = joint_state.configs.end() - 1;
      if (config->lim_pos.min == 0 && config->lim_pos.max == 0) RCLCPP_WARN(logger, "Joint %s has position limits of (0,0)", name.c_str());
      if (config->lim_vel.min == 0) RCLCPP_WARN(logger, "Joint %s has a 0 velocity limit", name.c_str());

      joint_state.state.name.push_back(name);
      joint_state.n_joints++;
    }

    // Setup joint state to have the right size
    joint_state.state.header.frame_id = model.getRoot()->name;
    joint_state.state.position = std::vector<double>(joint_state.n_joints, 0);
    joint_state.state.velocity = std::vector<double>(joint_state.n_joints, 0);
    joint_state.state.effort = std::vector<double>(0);
    // joint_state.state.effort = std::vector<double>(joint_state.n_joints, 0);
    joint_state.goals = std::vector<double>(joint_state.n_joints, 0);
  }

 private:
  std::string urdf_xml;
  data::ExtendedJointState joint_state;
  urdf::Model model;
};

}  // namespace simple_robots

#endif