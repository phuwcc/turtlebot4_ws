#include "tb4_multi_tools/controls.hpp"
#include <iostream>
#include <stdexcept>

void require(bool condition)
{
  if (!condition) {throw std::runtime_error("Control/origin invariant failed");}
}
int main()
{
  for (int i = -100; i <= 100; ++i) {
    const tb4_multi_tools::PlanarPose p{i * .17, i * -.31, i * .03};
    const auto inv = tb4_multi_tools::inverse(p);
    require(std::abs(std::cos(inv.yaw) * p.x - std::sin(inv.yaw) * p.y + inv.x) < 1e-10);
    require(std::abs(std::sin(inv.yaw) * p.x + std::cos(inv.yaw) * p.y + inv.y) < 1e-10);
    require(std::abs(inv.yaw + p.yaw) < 1e-10);
  }
  require(tb4_multi_tools::direction('i').linear == 1);
  require(tb4_multi_tools::direction(',').linear == -1);
  require(tb4_multi_tools::direction('j').angular == 1);
  require(tb4_multi_tools::direction('l').angular == -1);
  for (auto key : {'k', ' ', '\x03', '?'}) {
    const auto m = tb4_multi_tools::direction(key);
    require(m.linear == 0 && m.angular == 0);
  }
  const auto moving = tb4_multi_tools::direction('o');
  require(tb4_multi_tools::live_command(moving, .59, .6).linear == 1);
  require(tb4_multi_tools::live_command(moving, .6, .6).linear == 0);
  require(tb4_multi_tools::live_command(moving, 2., .6).angular == 0);
  std::cout << "Origin composition, teleop directions and timeout: OK\n";
}
