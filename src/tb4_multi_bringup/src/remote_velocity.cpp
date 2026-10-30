#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

using namespace std::chrono_literals;
class RemoteVelocity : public rclcpp::Node {
public:
  RemoteVelocity() : Node("remote_velocity") {
    mode_ = declare_parameter<std::string>("mode", "guard");
    timeout_ = declare_parameter<double>("timeout", 0.5);
    if ((mode_ != "guard" && mode_ != "stamp") || !std::isfinite(timeout_) || timeout_ <= 0.)
      throw std::invalid_argument("mode must be guard/stamp; timeout must be finite and positive");
    auto qos = rclcpp::QoS(1).best_effort();
    if (mode_ == "stamp") {
      stamped_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("remote_cmd_vel", qos);
      twist_sub_ = create_subscription<geometry_msgs::msg::Twist>("pc_cmd_vel", qos,
        [this](geometry_msgs::msg::Twist::ConstSharedPtr input) {
          geometry_msgs::msg::TwistStamped output;
          output.header.stamp = now(); output.twist = *input;
          stamped_pub_->publish(output);
        });
    } else {
      twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
      stamped_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("remote_cmd_vel", qos,
        [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr input) { receive(*input); });
      timer_ = create_wall_timer(50ms, [this] { tick(); });
    }
  }
  void stop() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    stopped_=true;
    if (twist_pub_ && rclcpp::ok()) twist_pub_->publish(geometry_msgs::msg::Twist{});
  }
private:
  void receive(const geometry_msgs::msg::TwistStamped& input) {
    const auto& s = input.header.stamp;
    if (s.nanosec >= 1000000000u) return;
    const int64_t stamp = int64_t(s.sec)*1000000000LL + s.nanosec;
    const double age = double(now().nanoseconds() - stamp)*1e-9;
    const auto& l = input.twist.linear; const auto& a = input.twist.angular;
    const std::array<double,6> values{{l.x,l.y,l.z,a.x,a.y,a.z}};
    if (!std::all_of(values.begin(),values.end(),[](double v){return std::isfinite(v);}) ||
        age < -0.1 || age > timeout_ || stamp <= last_stamp_) return;
    last_stamp_ = stamp;
    // Keep the original stamp's remaining lifetime; a delayed packet must not refresh it.
    expires_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(timeout_ - std::max(0.,age)));
    command_ = input.twist; received_ = true;
  }
  void tick() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    const bool active = !stopped_ && received_ && std::chrono::steady_clock::now() < expires_ &&
      count_publishers(stamped_sub_->get_topic_name()) == 1;
    twist_pub_->publish(active ? command_ : geometry_msgs::msg::Twist{});
  }
  std::mutex command_mutex_; bool stopped_{false};
  std::string mode_; double timeout_{}; bool received_{false};
  int64_t last_stamp_{std::numeric_limits<int64_t>::min()};
  std::chrono::steady_clock::time_point expires_;
  geometry_msgs::msg::Twist command_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv) {
  try {
    rclcpp::init(argc,argv);
    auto node=std::make_shared<RemoteVelocity>();
    node->get_node_base_interface()->get_context()->add_pre_shutdown_callback(
      [weak=std::weak_ptr<RemoteVelocity>(node)] { if (auto guard=weak.lock()) guard->stop(); });
    rclcpp::spin(node); node->stop();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("remote_velocity"),"%s",e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
