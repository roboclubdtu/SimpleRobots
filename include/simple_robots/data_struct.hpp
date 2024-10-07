#ifndef SIMPLE_ROBOTS_DATA_STRUCT
#define SIMPLE_ROBOTS_DATA_STRUCT

#include <map>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <vector>

namespace simple_robots::data {

using sensor_msgs::msg::JointState;

/**
 * @brief Structure to store the limits of a joint
 */
struct Limits {
  double min, max;
};

/**
 * @brief Structure to store a joint state
 *
 */
struct JointConfig {
  // Configuration
  std::string name;
  Limits lim_pos;
  double lim_vel, lim_eff;
};

struct ExtendedJointState {
  std::map<std::string, ulong> name_map;
  std::vector<JointConfig> configs;
  JointState state;
};

} // namespace simple_robots::data

#endif