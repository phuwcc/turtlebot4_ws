// Native replacement for the Python joint_state_publisher on the Pi.
// The TB4 URDF only has four non-fixed joints (wheels and wheel drops), and the
// stock publisher always sends them at position 0 at 10 Hz so robot_state_publisher
// can emit their TF. This node does the same with one preallocated message,
// avoiding the Python interpreter (~3.5% CPU and ~57 MB RSS per Pi).
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class WheelJointStates : public rclcpp::Node {
public:
  WheelJointStates() : Node("joint_state_publisher") {
    const auto names = declare_parameter<std::vector<std::string>>("joint_names", {
      "wheel_drop_left_joint", "left_wheel_joint", "wheel_drop_right_joint", "right_wheel_joint"});
    const double rate = declare_parameter<double>("rate", 10.);
    if (names.empty() || !std::isfinite(rate) || rate <= 0. || rate > 100.)
      throw std::invalid_argument("joint_names must be non-empty; rate must be in (0, 100] Hz");
    message_.name = names;
    message_.position.assign(names.size(), 0.);
    // Same QoS as joint_state_publisher (reliable, volatile, depth 10).
    publisher_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    timer_ = create_wall_timer(std::chrono::duration<double>(1. / rate), [this] {
      message_.header.stamp = now();
      publisher_->publish(message_);
    });
  }

private:
  sensor_msgs::msg::JointState message_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelJointStates>());
  rclcpp::shutdown();
  return 0;
}
