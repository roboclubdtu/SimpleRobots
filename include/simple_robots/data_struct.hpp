#ifndef SIMPLE_ROBOTS_DATA_STRUCT
#define SIMPLE_ROBOTS_DATA_STRUCT

#include <urdf/model.h>

#include <map>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <vector>

#include "simple_robots/common.hpp"

namespace simple_robots::data {

using sensor_msgs::msg::JointState;

/**
 * @brief Structure to store the limits of a joint
 */
struct Limits {
  double min, max;

  Limits() : min(0), max(0) {}
  Limits(double val) : min(-val), max(val) {}
  Limits(double _min, double _max) : min(_min), max(_max) {}

  inline bool inside(double val) { return math::in_bound(val, min, max); };
  inline double saturate(double val) { return math::saturate(val, min, max); }
};  // namespace simple_robots::data

struct JointType {
  enum Value {
    // Classic
    REVOLUTE = urdf::Joint::REVOLUTE,
    PRISMATIC = urdf::Joint::PRISMATIC,

    // Non standard
    CONTINUOUS = urdf::Joint::CONTINUOUS,
    PLANAR = urdf::Joint::PLANAR,
    FLOATING = urdf::Joint::FLOATING,
    FIXED = urdf::Joint::FIXED,

    UNKNOWN = urdf::Joint::UNKNOWN
  };

  JointType() : type(UNKNOWN) {}
  JointType(const urdf::Joint& j) : type(urdf2type(j.type)) {}
  JointType(std::shared_ptr<urdf::Joint> j) : type(urdf2type(j->type)) {}
  JointType(Value v) : type(v) {}
  operator Value() { return type; }

  static Value urdf2type(int type) { return static_cast<Value>(type); }
  static const char* getJointTypeName(JointType joint) {
    switch (joint.type) {
      case CONTINUOUS:
        return "continous";
      case FIXED:
        return "fixed";
      case FLOATING:
        return "floating";
      case PLANAR:
        return "planar";
      case PRISMATIC:
        return "prismatic";
      case REVOLUTE:
        return "revolute";
      default:
        return "unknown";
    }
  }

 public:
  Value type;
};

/**
 * @brief Structure to store a joint state
 *
 */
struct JointConfig {
  // Configuration
  std::string name;
  JointType type;
  Limits lim_pos;
  Limits lim_vel;
  Limits lim_eff;

  JointConfig(std::shared_ptr<urdf::Joint> j) {
    name = j->name;
    type = j;
    lim_pos = Limits(j->limits->lower, j->limits->upper);
    lim_vel = Limits(j->limits->velocity);
    lim_eff = Limits(j->limits->effort);
  }
};

enum class GoalType {
  VELOCITY = 1,
};

struct ExtendedJointState {
  std::map<std::string, ulong> name_map;
  std::vector<JointConfig> configs;
  GoalType goal = GoalType::VELOCITY;
  ulong n_joints;
  std::vector<double> goals;

  JointState state;
};

}  // namespace simple_robots::data

#endif