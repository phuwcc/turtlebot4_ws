#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "tb4_multi_tools/controls.hpp"
#include <termios.h>
#include <poll.h>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <stdexcept>

class Terminal
{
public:
  Terminal()
  {
    if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &saved_) != 0) {
      throw std::runtime_error("Use an interactive terminal (SSH: ssh -t).");
    }
    auto raw = saved_;
    cfmakeraw(&raw);
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
      throw std::runtime_error("Cannot configure terminal.");
    }
  }
  ~Terminal() {tcsetattr(STDIN_FILENO, TCSANOW, &saved_);}
private:
  termios saved_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("teleop_keyboard_cpp");
  const bool remote = node->declare_parameter("remote", false);
  auto publisher = remote ? nullptr : node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  auto stamped = remote ? node->create_publisher<geometry_msgs::msg::TwistStamped>(
    "remote_cmd_vel", rclcpp::QoS(1).best_effort()) : nullptr;
  auto publish = [&](const geometry_msgs::msg::Twist & command) {
      if (remote) {
        geometry_msgs::msg::TwistStamped message;
        message.header.stamp = node->now();
        message.twist = command;
        stamped->publish(message);
      } else {publisher->publish(command);}
    };
  try {
    double speed = node->declare_parameter("speed", 0.08);
    double turn = node->declare_parameter("turn", 0.3);
    const auto timeout = node->declare_parameter("key_timeout", 0.6);
    if (!std::isfinite(speed) || !std::isfinite(turn) || !std::isfinite(timeout) ||
      speed < 0 || turn < 0 || timeout <= 0)
    {
      throw std::runtime_error("speed/turn must be nonnegative; key_timeout must be positive.");
    }
    std::cout << "C++ teleop: i/, forward/back; j/l turn; u/o/m/. arc; k/Space stop.\n"
      "q/z both speeds; w/x linear; e/c angular. Ctrl+C exits.\n"
      "Hold a motion key to repeat. Stops after " << timeout << " s without a motion key.\n"
      << std::flush;
    Terminal terminal;
    tb4_multi_tools::Motion motion;
    auto last_key = std::chrono::steady_clock::now() - std::chrono::seconds(2);
    rclcpp::WallRate rate(20);
    bool stopped = false;
    while (rclcpp::ok() && !stopped) {
      pollfd input{STDIN_FILENO, POLLIN, 0};
      while (poll(&input, 1, 0) > 0) {
        if (input.revents & (POLLHUP | POLLERR | POLLNVAL)) {stopped = true; break;}
        char key = 0;
        if (!(input.revents & POLLIN) || read(STDIN_FILENO, &key, 1) != 1) {
          stopped = true; break;
        }
        if (key == 3) {stopped = true; break;}
        if (key == 'q' || key == 'z') {
          const auto factor = key == 'q' ? 1.1 : 0.9; speed *= factor; turn *= factor;
        } else if (key == 'w' || key == 'x') {
          speed *= key == 'w' ? 1.1 : 0.9;
        } else if (key == 'e' || key == 'c') {
          turn *= key == 'e' ? 1.1 : 0.9;
        } else {
          motion = tb4_multi_tools::direction(key);
          last_key = std::chrono::steady_clock::now();
        }
      }
      const double age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_key).count();
      const auto active = stopped ? tb4_multi_tools::Motion{} : tb4_multi_tools::live_command(motion, age, timeout);
      geometry_msgs::msg::Twist command;
      command.linear.x = active.linear * speed;
      command.angular.z = active.angular * turn;
      publish(command);
      rclcpp::spin_some(node);
      if (rclcpp::ok()) {rate.sleep();}
    }
    if (rclcpp::ok()) {publish(geometry_msgs::msg::Twist{});}
  } catch (const std::exception & e) {
    if (rclcpp::ok()) {publish(geometry_msgs::msg::Twist{});}
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return 1;
  }
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return 0;
}
