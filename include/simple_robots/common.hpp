#ifndef SIMPLE_ROBOTS_COMMON_DEF
#define SIMPLE_ROBOTS_COMMON_DEF

#include <memory>
#include <vector>

namespace simple_robots {
#define constchar static constexpr const char*
#define constval static constexpr
typedef unsigned long ulong;

template <typename T>
using sptr = std::shared_ptr<T>;

namespace math {
template <typename T>
inline bool in_bound(T val, T bound) {
  return (bound < val && val < bound);
}

template <typename T>
inline bool in_bound(T val, T min, T max) {
  return (min < val && val < max);
}

template <typename T>
inline bool in_bound(T val, T min, T max, T offset) {
  return (min + offset < val && val < max + offset);
}

template <typename T>
inline T saturate(T val, T bound) {
  return (val < -bound) ? bound : ((val > bound) ? bound : val);
}

template <typename T>
inline T saturate(T val, T min, T max) {
  return (val < min) ? min : ((val > max) ? max : val);
}

template <typename T>
inline T saturate(T val, T min, T max, T offset) {
  return (val < min + offset) ? min + offset : ((val > max + offset) ? max + offset : val);
}

}  // namespace math

}  // namespace simple_robots

#endif