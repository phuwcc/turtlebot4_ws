#pragma once
#include <cmath>

namespace tb4_multi_tools
{
struct PlanarPose {double x, y, yaw;};
inline PlanarPose inverse(const PlanarPose & p)
{
  const auto c = std::cos(p.yaw), s = std::sin(p.yaw);
  return {-c * p.x - s * p.y, s * p.x - c * p.y, -p.yaw};
}
struct Motion {double linear = 0., angular = 0.;};
inline Motion direction(char key)
{
  switch (key) {
    case 'i': return {1, 0};
    case ',': return {-1, 0};
    case 'j': return {0, 1};
    case 'l': return {0, -1};
    case 'u': return {1, 1};
    case 'o': return {1, -1};
    case 'm': return {-1, -1};
    case '.': return {-1, 1};
    default: return {0, 0};
  }
}
inline Motion live_command(Motion m, double elapsed, double timeout)
{
  return elapsed < timeout ? m : Motion{};
}
}  // namespace tb4_multi_tools
