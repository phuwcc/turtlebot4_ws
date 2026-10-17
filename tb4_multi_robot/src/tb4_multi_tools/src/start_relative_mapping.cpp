#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "tb4_multi_tools/controls.hpp"

#include <sys/file.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

class RelativeMapping : public rclcpp::Node
{
public:
  RelativeMapping() : Node("mapping_start_origin")
  {
    params_ = declare_parameter("slam_params_file", std::string(""));
    if (params_.empty()) {throw std::runtime_error("slam_params_file is required");}
    std::string key = get_namespace();
    std::replace(key.begin(), key.end(), '/', '_');
    origin_file_ = declare_parameter("origin_file", "/tmp/tb4_mapping" + key + ".json");
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    mapping_frame_ = declare_parameter("mapping_frame", std::string("mapping_odom"));
    map_frame_ = declare_parameter("map_frame", std::string("map"));
    const auto odom_topic = declare_parameter("odom_topic", std::string("odom"));
    for (const auto & frame : {odom_frame_, base_frame_, mapping_frame_, map_frame_}) {
      if (frame.empty() || frame.front() == '/' || frame.find_first_of("\"\\\n\r") != std::string::npos) {
        throw std::runtime_error("Use nonempty relative TF frame IDs");
      }
    }
    timeout_ = declare_parameter("capture_timeout", 25.0);
    const bool sync = declare_parameter("sync", false);
    executable_ = ament_index_cpp::get_package_prefix("slam_toolbox") +
      "/lib/slam_toolbox/" + (sync ? "sync_slam_toolbox_node" : "async_slam_toolbox_node");
    if (timeout_ <= 0 || !std::isfinite(timeout_)) {
      throw std::runtime_error("capture_timeout must be positive and finite");
    }
    lock_ = open((origin_file_ + ".lock").c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0600);
    if (lock_ < 0) {throw std::runtime_error("Cannot open mapping lock file");}
    if (flock(lock_, LOCK_EX | LOCK_NB) != 0) {
      close(lock_); lock_ = -1;
      throw std::runtime_error("A relative mapping session is already running");
    }
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_, this, false);
    broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        odom_ = msg; received_ = Clock::now();
      });
    started_ = Clock::now();
    timer_ = create_wall_timer(50ms, [this]() {capture();});
    RCLCPP_INFO(get_logger(), "Keep the robot still. Waiting for fresh odom and TF...");
  }

  ~RelativeMapping() override
  {
    stop_child();
    if (lock_ >= 0) {close(lock_);}
  }

  int exit_code() const {return exit_code_;}

private:
  void fail(const std::string & message)
  {
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    exit_code_ = 1;
    rclcpp::shutdown();
  }

  void capture()
  {
    const auto now = Clock::now();
    if (now - started_ > std::chrono::duration<double>(timeout_)) {
      fail("No fresh stationary odom/TF before timeout. SLAM was not started."); return;
    }
    if (now - graph_checked_ >= 1s) {
      graph_checked_ = now;
      for (const auto & name : get_node_names()) {
        const auto prefix = std::string(get_namespace()) == "/" ? "/" : std::string(get_namespace()) + "/";
        if (name == prefix + "slam_toolbox" || name == prefix + "amcl") {
          fail("Stop the existing SLAM/AMCL session first."); return;
        }
      }
    }
    if (!odom_ || now - received_ > 500ms ||
      std::hypot(odom_->twist.twist.linear.x, odom_->twist.twist.linear.y) > 0.01 ||
      std::abs(odom_->twist.twist.angular.z) > 0.02)
    {
      still_ = false; return;
    }
    if (!still_) {still_ = true; still_since_ = now;}
    if (now - still_since_ < 1s) {return;}
    geometry_msgs::msg::TransformStamped initial;
    try {initial = buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);}
    catch (const tf2::TransformException &) {return;}
    const double age = (get_clock()->now() - rclcpp::Time(initial.header.stamp)).seconds();
    if (age < -0.1 || age > 0.5) {return;}
    const auto & t = initial.transform.translation;
    const auto & q = initial.transform.rotation;
    const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y),
      1 - 2 * (q.y * q.y + q.z * q.z));
    if (!std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(yaw)) {
      fail("Invalid initial TF pose"); return;
    }
    const auto anchor = tb4_multi_tools::inverse({t.x, t.y, yaw});
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = mapping_frame_;
    msg.child_frame_id = odom_frame_;
    msg.transform.translation.x = anchor.x;
    msg.transform.translation.y = anchor.y;
    msg.transform.rotation.z = std::sin(anchor.yaw / 2);
    msg.transform.rotation.w = std::cos(anchor.yaw / 2);
    std::ofstream record(origin_file_);
    if (!record) {fail("Cannot write origin file: " + origin_file_); return;}
    record << std::setprecision(17) << "{\n  \"source_frame\": \"" << odom_frame_ << "\",\n"
      "  \"base_frame\": \"" << base_frame_ << "\",\n  \"captured_x\": " << t.x <<
      ",\n  \"captured_y\": " << t.y << ",\n  \"captured_yaw_rad\": " << yaw <<
      ",\n  \"mapping_odom_to_odom\": {\"x\": " << anchor.x << ", \"y\": " << anchor.y <<
      ", \"yaw_rad\": " << anchor.yaw << "}\n}\n";
    record.close();
    if (!record) {fail("Failed to save mapping origin"); return;}
    broadcaster_->sendTransform(msg);
    RCLCPP_INFO(get_logger(),
      "Start origin captured: (%.3f, %.3f, %.3f rad) -> mapping (0,0,0). "
      "Keep still until SLAM registers the laser.", t.x, t.y, yaw);

    // Prepare argv before fork: the child only performs async-signal-safe calls.
    std::vector<std::string> args{executable_, "--ros-args", "-r", "__node:=slam_toolbox",
      "-r", "__ns:=" + std::string(get_namespace()),
      "--params-file", params_, "-p", "odom_frame:=" + mapping_frame_,
      "-p", "base_frame:=" + base_frame_, "-p", "map_frame:=" + map_frame_,
      "-p", "use_sim_time:=false", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static",
      "-r", "/scan:=scan", "-r", "/map:=map", "-r", "/map_metadata:=map_metadata"};
    std::vector<char *> argv;
    for (auto & arg : args) {argv.push_back(arg.data());}
    argv.push_back(nullptr);
    child_ = fork();
    if (child_ == 0) {
      setpgid(0, 0);
      execv(argv[0], argv.data());
      _exit(127);
    }
    if (child_ < 0) {fail("Cannot start SLAM process"); return;}
    setpgid(child_, child_);
    // No odom/TF processing is needed after the one-time capture.
    odom_sub_.reset(); odom_.reset(); listener_.reset(); buffer_.reset();
    timer_->cancel();
    timer_ = create_wall_timer(500ms, [this]() {
      int status = 0;
      const auto result = waitpid(child_, &status, WNOHANG);
      if (result == child_) {
        child_ = -1;
        exit_code_ = WIFEXITED(status) ? WEXITSTATUS(status) : 1;
        RCLCPP_INFO(get_logger(), "SLAM exited; stopping mapping origin node.");
        rclcpp::shutdown();
      }
    });
  }

  void stop_child()
  {
    if (child_ <= 0) {return;}
    int status = 0;
    if (waitpid(child_, &status, WNOHANG) != 0) {child_ = -1; return;}
    for (const auto sig : {SIGINT, SIGTERM, SIGKILL}) {
      kill(-child_, sig);
      const auto until = Clock::now() + (sig == SIGINT ? 8s : 2s);
      while (Clock::now() < until) {
        if (waitpid(child_, &status, WNOHANG) != 0) {child_ = -1; return;}
        std::this_thread::sleep_for(50ms);
      }
    }
    waitpid(child_, &status, 0); child_ = -1;
  }

  std::string params_, executable_, origin_file_;
  std::string odom_frame_, base_frame_, mapping_frame_, map_frame_;
  double timeout_ = 25.;
  int lock_ = -1, exit_code_ = 0;
  pid_t child_ = -1;
  bool still_ = false;
  Clock::time_point started_, received_, still_since_, graph_checked_{};
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 1;
  try {
    auto node = std::make_shared<RelativeMapping>();
    rclcpp::spin(node);
    result = node->exit_code();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("mapping_start_origin"), "%s", e.what());
  }
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return result;
}
