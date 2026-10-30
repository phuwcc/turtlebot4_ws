#include "fleet_runtime.hpp"
int main(int argc, char** argv) {
  try {
    rclcpp::init(argc,argv);
    auto node=make_fleet_node(TB4_SOCKET_ENABLED != 0);
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("fleet_view"),"%s",e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
