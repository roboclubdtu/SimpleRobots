#ifndef SIMPLE_ROBOT_URDF
#define SIMPLE_ROBOT_URDF

#include "simple_robots/data_struct.hpp"
#include <iostream>
#include <rclcpp/logging.hpp>
#include <string>
#include <urdf/model.h>

namespace simple_robots {

class URDFLoader {
public:
  typedef std::shared_ptr<URDFLoader> SharedPtr;

  URDFLoader() {}
  URDFLoader(const std::string &urdf_xml, const rclcpp::Logger &logger)
      : urdf_xml(urdf_xml) {
    if (urdf_xml.empty())
      throw std::runtime_error("The URDF xml shouldn't be empty.");
    extract_extended_joint_state(logger);
  }

  static SharedPtr make(const std::string &urdf_xml,
                        const rclcpp::Logger &logger) {
    return std::make_shared<URDFLoader>(urdf_xml, logger);
  }

  data::ExtendedJointState getExtendedJointState() const {
    return joint_state;
  };

private:
  void extract_extended_joint_state(const rclcpp::Logger &logger) {
    if (!model.initString(urdf_xml))
      throw std::runtime_error(
          "Couldn't initialize URDF model from XML string");

    // Parse joints
    size_t n_joints = 0;
    for (auto const &[name, joint] : model.joints_) {
      auto type = joint->type;

      RCLCPP_INFO(logger, "Found joint %s of type %s", name.c_str(),
                  URDFLoader::getJointTypeName(type));
      if (!(type == urdf::Joint::REVOLUTE || type == urdf::Joint::PRISMATIC))
        continue;

      // Import it in the extended joint state member
      joint_state.name_map.insert_or_assign(name, joint_state.configs.size());
      joint_state.configs.push_back(data::JointConfig{
          name, data::Limits{joint->limits->lower, joint->limits->upper},
          joint->limits->velocity, joint->limits->effort});
      joint_state.state.name.push_back(name);
      n_joints++;
    }

    // Setup joint state to have the right size
    joint_state.state.header.frame_id = model.getRoot()->name;
    joint_state.state.position = std::vector<double>(n_joints);
    joint_state.state.velocity = std::vector<double>(n_joints);
    joint_state.state.effort = std::vector<double>(n_joints);
  }

  static inline const char *getJointTypeName(int type) {
    switch (type) {
    case urdf::Joint::CONTINUOUS:
      return "continous";
    case urdf::Joint::FIXED:
      return "fixed";
    case urdf::Joint::FLOATING:
      return "floating";
    case urdf::Joint::PLANAR:
      return "planar";
    case urdf::Joint::PRISMATIC:
      return "prismatic";
    case urdf::Joint::REVOLUTE:
      return "revolute";
    default:
      return "unknown";
    }
  }

private:
  std::string urdf_xml;
  data::ExtendedJointState joint_state;
  urdf::Model model;
};

} // namespace simple_robots

#endif