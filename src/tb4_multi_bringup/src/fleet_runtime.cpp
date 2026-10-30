#include "fleet_runtime.hpp"
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;
using Json=nlohmann::json;
using Goal=nav2_msgs::action::NavigateToPose;
using GoalHandle=rclcpp_action::ClientGoalHandle<Goal>;
using Pose=geometry_msgs::msg::Pose;
using PoseStamped=geometry_msgs::msg::PoseStamped;
using InitialPose=geometry_msgs::msg::PoseWithCovarianceStamped;
using TF=tf2_msgs::msg::TFMessage;
using Trigger=std_srvs::srv::Trigger;
using Clock=std::chrono::steady_clock;
static const std::array<std::string,3> ROBOTS{{"robot1","robot2","robot3"}};
static const float COLORS[3][3]={{.2f,.8f,1.f},{1.f,.55f,.1f},{.3f,1.f,.3f}};

static std::string display_frame(const std::string& robot,std::string frame,const std::string& map) {
  const auto begin=frame.find_first_not_of('/');
  if (begin==std::string::npos) return "";
  frame.erase(0,begin);
  if (frame==map) return "map";
  return frame.compare(0,robot.size()+1,robot+"/")==0 ? frame : robot+"/"+frame;
}
static bool valid_pose(const Pose& p) {
  const std::array<double,7> v{{p.position.x,p.position.y,p.position.z,
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w}};
  const double norm=v[3]*v[3]+v[4]*v[4]+v[5]*v[5]+v[6]*v[6];
  return std::all_of(v.begin(),v.end(),[](double n){return std::isfinite(n);}) && std::abs(norm-1.)<.01;
}

class FleetRuntime final : public rclcpp::Node {
  struct Robot {
    std::string map,base;
    rclcpp_action::Client<Goal>::SharedPtr action;
    rclcpp::Publisher<InitialPose>::SharedPtr pose_pub;
    GoalHandle::SharedPtr goal;
    bool busy{false},cancel_requested{false};
    Json last_result=nullptr;
    uint64_t generation{0};
  };
  struct Client {
    int fd{-1};
    std::string input,output;
    size_t sent{0};
    Clock::time_point deadline;
  };
public:
  explicit FleetRuntime(bool with_socket):Node("fleet_view") {
    scan_period_=1.0/declare_parameter<double>("visualization_scan_hz",5.0);
    if (!std::isfinite(scan_period_) || scan_period_<=0.) throw std::invalid_argument("visualization_scan_hz must be positive and finite");
    const int count=declare_parameter<int>("robot_count",3);
    const auto ids=declare_parameter<std::string>("robot_ids","");
    if (count<1 || count>3) throw std::invalid_argument("robot_count must be 1, 2 or 3");
    if (ids.empty()) robots_.assign(ROBOTS.begin(),ROBOTS.begin()+count);
    else {
      std::stringstream stream(ids); std::string id;
      while (std::getline(stream,id,',')) robots_.push_back(id);
      if (ids.back()==',' || int(robots_.size())!=count ||
          !std::is_sorted(robots_.begin(),robots_.end()) ||
          std::adjacent_find(robots_.begin(),robots_.end())!=robots_.end())
        throw std::invalid_argument("robot_ids must contain unique, ordered active robots");
      for (const auto& r:robots_) if (std::find(ROBOTS.begin(),ROBOTS.end(),r)==ROBOTS.end())
        throw std::invalid_argument("robot_ids contains an unknown robot");
    }
    selected_=declare_parameter<std::string>("selected_robot",robots_.front());
    if (!active(selected_)) throw std::invalid_argument("selected_robot must be active");
    for (const auto& r:robots_) {
      Robot state;
      state.map=declare_parameter<std::string>(r+".map_frame","map");
      state.base=declare_parameter<std::string>(r+".base_frame","base_link");
      state.action=rclcpp_action::create_client<Goal>(this,"/"+r+"/navigate_to_pose");
      state.pose_pub=create_publisher<InitialPose>("/"+r+"/initialpose",10);
      states_.emplace(r,std::move(state));
    }
    parameter_callback_=add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& parameters) {
      rcl_interfaces::msg::SetParametersResult result; result.successful=true;
      for (const auto& p:parameters) {
        if (p.get_name()=="selected_robot") {
          if (p.get_type()!=rclcpp::ParameterType::PARAMETER_STRING || !active(p.as_string())) {
            result.successful=false; result.reason="selected_robot must be active"; return result;
          }
        } else if (p.get_name()=="robot_count" || p.get_name()=="robot_ids" ||
                   ends_with(p.get_name(),".map_frame") || ends_with(p.get_name(),".base_frame")) {
          result.successful=false; result.reason="Frame settings require restarting the launch"; return result;
        }
      }
      // The executor is single threaded; update only after all validation succeeds.
      for (const auto& p:parameters) if (p.get_name()=="selected_robot") selected_=p.as_string();
      return result;
    });
    const auto latched=rclcpp::QoS(1).transient_local();
    const auto live=rclcpp::QoS(10).best_effort();
    tf_pub_=create_publisher<TF>("/tb4_view/tf",100);
    static_pub_=create_publisher<TF>("/tb4_view/tf_static",latched);
    labels_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("/tb4_view/labels",latched);
    for (const auto& r:robots_) {
      auto tf_sub=create_subscription<TF>("/"+r+"/tf",rclcpp::QoS(100).best_effort(),
        [this,r](TF::UniquePtr message) { adapt(*message,r); tf_pub_->publish(std::move(message)); });
      subscriptions_.push_back(tf_sub);
      auto static_sub=create_subscription<TF>("/"+r+"/tf_static",rclcpp::QoS(100).transient_local(),
        [this,r](TF::UniquePtr message) {
          adapt(*message,r);
          for (auto& t:message->transforms) static_cache_[t.child_frame_id]=std::move(t);
          TF combined; combined.transforms.reserve(static_cache_.size());
          for (const auto& entry:static_cache_) combined.transforms.push_back(entry.second);
          static_pub_->publish(combined);
        });
      subscriptions_.push_back(static_sub);
      relay<sensor_msgs::msg::LaserScan>(r,"scan",rclcpp::QoS(1).best_effort());
      relay<nav_msgs::msg::OccupancyGrid>(r,"map",latched);
      relay<InitialPose>(r,"amcl_pose",live);
      relay<nav_msgs::msg::Path>(r,"plan",live);
      relay<nav_msgs::msg::OccupancyGrid>(r,"global_costmap/costmap",latched);
      relay<nav_msgs::msg::OccupancyGrid>(r,"local_costmap/costmap",latched);
      relay<geometry_msgs::msg::PolygonStamped>(r,"local_costmap/published_footprint",live);
    }
    subscriptions_.push_back(create_subscription<InitialPose>("/tb4_view/initialpose",10,
      [this](InitialPose::UniquePtr p) {
        if (p->header.frame_id=="map" && valid_pose(p->pose.pose)) initial_pose(*p,selected_);
        else RCLCPP_ERROR(get_logger(),"Pose rejected: use frame map and a finite normalized pose");
      }));
    subscriptions_.push_back(create_subscription<PoseStamped>("/tb4_view/goal_pose",10,
      [this](PoseStamped::UniquePtr p) {
        if (p->header.frame_id=="map" && valid_pose(p->pose)) goal_pose(*p,selected_);
        else RCLCPP_ERROR(get_logger(),"Goal rejected: use frame map and a finite normalized pose");
      }));
    cancel_service_=create_service<Trigger>("/tb4_view/cancel",
      [this](const Trigger::Request::SharedPtr,Trigger::Response::SharedPtr out) {
        out->success=cancel_robot(selected_); out->message=out->success ? "Cancellation requested" : "No owned goal";
      });
    label_timer_=create_wall_timer(1000ms,[this]{publish_labels();});
    if (with_socket) open_socket(declare_parameter<int>("socket_port",8765));
    RCLCPP_INFO(get_logger(),"Native fleet view ready for %zu robot(s), selected %s",robots_.size(),selected_.c_str());
  }
  ~FleetRuntime() override {
    for (auto& client:clients_) if (client.fd>=0) ::close(client.fd);
    if (listen_fd_>=0) ::close(listen_fd_);
  }
private:
  static bool ends_with(const std::string& value,const std::string& suffix) {
    return value.size()>=suffix.size() && value.compare(value.size()-suffix.size(),suffix.size(),suffix)==0;
  }
  bool active(const std::string& robot) const {
    return std::find(robots_.begin(),robots_.end(),robot)!=robots_.end();
  }
  template<class T> void adapt(T& message,const std::string& robot) {
    message.header.frame_id=display_frame(robot,message.header.frame_id,states_.at(robot).map);
  }
  void adapt(nav_msgs::msg::Path& message,const std::string& robot) {
    message.header.frame_id=display_frame(robot,message.header.frame_id,states_.at(robot).map);
    for (auto& pose:message.poses) pose.header.frame_id=display_frame(robot,pose.header.frame_id,states_.at(robot).map);
  }
  void adapt(TF& message,const std::string& robot) {
    for (auto& t:message.transforms) {
      t.header.frame_id=display_frame(robot,t.header.frame_id,states_.at(robot).map);
      t.child_frame_id=display_frame(robot,t.child_frame_id,states_.at(robot).map);
    }
  }
  template<class T> void relay(const std::string& robot,const std::string& topic,const rclcpp::QoS& qos) {
    auto pub=create_publisher<T>("/tb4_view/"+robot+"/"+topic,qos);
    publishers_.push_back(pub);
    subscriptions_.push_back(create_subscription<T>("/"+robot+"/"+topic,qos,
      [this,robot,topic,pub](typename T::UniquePtr message) {
        // Cap only the display scan; localization and obstacle layers still use raw /robotN/scan.
        if (topic=="scan") {
          const auto tick=Clock::now(); auto found=last_scan_.find(robot);
          if (found!=last_scan_.end() && std::chrono::duration<double>(tick-found->second).count()<scan_period_) return;
          last_scan_[robot]=tick;
        }
        // Mutate the received message in place, avoiding Python's full deepcopy of map/scan data.
        adapt(*message,robot); pub->publish(std::move(message));
      }));
  }
  bool initial_pose(InitialPose message,const std::string& robot) {
    auto& state=states_.at(robot);
    if (state.busy) return false;
    message.header.frame_id=state.map; message.header.stamp=now();
    state.pose_pub->publish(message);
    RCLCPP_INFO(get_logger(),"Initial pose -> %s",robot.c_str()); return true;
  }
  bool goal_pose(PoseStamped message,const std::string& robot) {
    auto& state=states_.at(robot);
    if (state.busy || !state.action->action_server_is_ready()) return false;
    Goal::Goal goal; goal.pose=std::move(message); goal.pose.header.frame_id=state.map; goal.pose.header.stamp=now();
    state.busy=true; state.last_result=nullptr; state.cancel_requested=false;
    const auto generation=++state.generation;
    typename rclcpp_action::Client<Goal>::SendGoalOptions options;
    options.goal_response_callback=[this,robot,generation](GoalHandle::SharedPtr handle) {
      auto& s=states_.at(robot); if (s.generation!=generation) return;
      if (!handle) {s.busy=false;s.last_result="rejected";return;}
      s.goal=handle;
      if (s.cancel_requested) request_cancel(robot);
    };
    options.result_callback=[this,robot,generation](const GoalHandle::WrappedResult& result) {
      auto& s=states_.at(robot); if (s.generation!=generation) return;
      s.goal.reset(); s.busy=false; s.cancel_requested=false;
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:s.last_result="succeeded";break;
        case rclcpp_action::ResultCode::CANCELED:s.last_result="canceled";break;
        case rclcpp_action::ResultCode::ABORTED:s.last_result="aborted";break;
        default:s.last_result="unknown";
      }
    };
    try {state.action->async_send_goal(goal,options);return true;}
    catch (const std::exception& e) {
      state.busy=false;state.last_result="error"; RCLCPP_ERROR(get_logger(),"%s",e.what());return false;
    }
  }
  bool cancel_robot(const std::string& robot) {
    auto& state=states_.at(robot);
    if (!state.busy) return false;
    state.cancel_requested=true;
    if (state.goal) request_cancel(robot);
    return true;
  }
  void request_cancel(const std::string& robot) {
    auto& state=states_.at(robot);
    try {state.action->async_cancel_goal(state.goal);}
    catch (const std::exception& e) {RCLCPP_WARN(get_logger(),"Cancel failed: %s",e.what());}
  }
  Json dispatch(const Json& request) {
    if (!request.is_object()) throw std::invalid_argument("Expected JSON object");
    if (!request.contains("command") || !request["command"].is_string())
      throw std::invalid_argument("command must be a string");
    const auto command=request["command"].get<std::string>();
    if (command=="status") {
      Json robots=Json::object();
      for (const auto& robot:robots_) {
        const auto& s=states_.at(robot);
        robots[robot]={{"nav_ready",s.action->action_server_is_ready()},{"busy",s.busy},
          {"accepted",bool(s.goal)},{"cancel_requested",s.cancel_requested},{"last_result",s.last_result}};
      }
      return {{"ok",true},{"selected_robot",selected_},{"robots",robots}};
    }
    if (!request.contains("robot") || !request["robot"].is_string() || !active(request["robot"].get<std::string>()))
      throw std::invalid_argument("robot must be active");
    const auto robot=request["robot"].get<std::string>();
    auto& state=states_.at(robot);
    if (command=="select") {
      auto result=set_parameter(rclcpp::Parameter("selected_robot",robot));
      return {{"ok",result.successful},{"selected_robot",robot},{"message",result.reason}};
    }
    if (command=="cancel") {
      bool ok=cancel_robot(robot);return {{"ok",ok},{"message",ok ? "Cancellation requested" : "No owned goal"}};
    }
    if (command!="pose" && command!="goal") throw std::invalid_argument("Unknown command");
    std::array<double,3> values{};
    const char* keys[]={"x","y","yaw"};
    for (size_t i=0;i<3;++i) {
      if (!request.contains(keys[i])) {if (i==2) continue;throw std::invalid_argument("x and y are required");}
      if (!request[keys[i]].is_number()) throw std::invalid_argument("Coordinates must be finite numbers");
      values[i]=request[keys[i]].get<double>();
      if (!std::isfinite(values[i])) throw std::invalid_argument("Coordinates must be finite numbers");
    }
    if (state.busy) return {{"ok",false},{"reason","robot_busy"},{"message","Cancel the active goal first"}};
    if (command=="goal" && !state.action->action_server_is_ready())
      return {{"ok",false},{"reason","nav_unavailable"},{"message","Nav2 action server unavailable"}};
    PoseStamped pose; pose.header.frame_id=state.map;pose.header.stamp=now();
    pose.pose.position.x=values[0];pose.pose.position.y=values[1];
    pose.pose.orientation.z=std::sin(values[2]/2);pose.pose.orientation.w=std::cos(values[2]/2);
    bool ok;
    if (command=="pose") {
      InitialPose initial;initial.header=pose.header;initial.pose.pose=pose.pose;
      initial.pose.covariance[0]=initial.pose.covariance[7]=.25;initial.pose.covariance[35]=.06854;
      ok=initial_pose(initial,robot);
    } else ok=goal_pose(pose,robot);
    return {{"ok",ok},{"message",command=="pose" ? "Initial pose published" : "Goal submitted; poll status for result"}};
  }
  void open_socket(int port) {
    if (port<0 || port>65535) throw std::invalid_argument("socket_port must be 0..65535");
    listen_fd_=::socket(AF_INET,SOCK_STREAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
    if (listen_fd_<0) throw std::runtime_error("socket: "+std::string(std::strerror(errno)));
    int yes=1;::setsockopt(listen_fd_,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(yes));
    sockaddr_in address{};address.sin_family=AF_INET;address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);address.sin_port=htons(port);
    if (::bind(listen_fd_,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0 || ::listen(listen_fd_,32)<0) {
      const auto error=std::string(std::strerror(errno));::close(listen_fd_);listen_fd_=-1;
      throw std::runtime_error("Task socket: "+error);
    }
    socklen_t size=sizeof(address);::getsockname(listen_fd_,reinterpret_cast<sockaddr*>(&address),&size);
    socket_timer_=create_wall_timer(20ms,[this]{poll_socket();});
    RCLCPP_INFO(get_logger(),"Task socket listening on 127.0.0.1:%u",ntohs(address.sin_port));
  }
  void poll_socket() {
    // Every operation is nonblocking and bounded. ROS action callbacks, TCP commands and
    // RViz requests share this executor, so no socket thread can race goal ownership.
    for (size_t count=0;count<32;++count) {
      const int fd=::accept4(listen_fd_,nullptr,nullptr,SOCK_NONBLOCK|SOCK_CLOEXEC);
      if (fd<0) break;
      if (clients_.size()>=32) {::close(fd);continue;}
      clients_.push_back(Client{fd,"","",0,Clock::now()+3s});
    }
    for (auto& client:clients_) {
      if (Clock::now()>client.deadline) {::close(client.fd);client.fd=-1;continue;}
      if (client.output.empty()) {
        char buffer[8193];
        const auto length=::recv(client.fd,buffer,sizeof(buffer),0);
        if (length>0) client.input.append(buffer,size_t(length));
        else if (length==0 || (errno!=EAGAIN && errno!=EWOULDBLOCK && errno!=EINTR)) {
          ::close(client.fd);client.fd=-1;continue;
        }
        const auto newline=client.input.find('\n');
        if (newline!=std::string::npos || client.input.size()>8192) {
          Json response;
          try {
            if (newline==std::string::npos || newline+1>8192)
              throw std::invalid_argument("Maximum request size is 8192 bytes including newline");
            response=dispatch(Json::parse(client.input.substr(0,newline)));
          } catch (const std::exception& e) {response={{"ok",false},{"error",e.what()}};}
          client.output=response.dump()+"\n";
        }
      }
      if (!client.output.empty()) {
        const auto sent=::send(client.fd,client.output.data()+client.sent,client.output.size()-client.sent,MSG_NOSIGNAL);
        if (sent>0) client.sent+=size_t(sent);
        if (client.sent==client.output.size() ||
            (sent<0 && errno!=EAGAIN && errno!=EWOULDBLOCK && errno!=EINTR)) {
          ::close(client.fd);client.fd=-1;
        }
      }
    }
    clients_.erase(std::remove_if(clients_.begin(),clients_.end(),[](const Client& c){return c.fd<0;}),clients_.end());
  }
  void publish_labels() {
    visualization_msgs::msg::MarkerArray labels;
    for (size_t i=0;i<robots_.size();++i) {
      const auto& robot=robots_[i];
      const auto color=std::distance(ROBOTS.begin(),std::find(ROBOTS.begin(),ROBOTS.end(),robot));
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id=display_frame(robot,states_.at(robot).base,states_.at(robot).map);
      marker.ns="robots";marker.id=int(i);marker.type=visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.pose.orientation.w=1.;marker.pose.position.z=.65;marker.scale.z=.22;
      marker.color.r=COLORS[color][0];marker.color.g=COLORS[color][1];marker.color.b=COLORS[color][2];marker.color.a=1.;
      marker.text=robot+(robot==selected_ ? " [SELECTED]" : "");labels.markers.push_back(marker);
    }
    visualization_msgs::msg::Marker marker;marker.header.frame_id="map";marker.ns="selection";marker.id=0;
    marker.type=visualization_msgs::msg::Marker::TEXT_VIEW_FACING;marker.pose.orientation.w=1.;
    marker.pose.position.y=-.7;marker.pose.position.z=.1;marker.scale.z=.22;
    const auto color=std::distance(ROBOTS.begin(),std::find(ROBOTS.begin(),ROBOTS.end(),selected_));
    marker.color.r=COLORS[color][0];marker.color.g=COLORS[color][1];marker.color.b=COLORS[color][2];marker.color.a=1.;
    marker.text="Pose / Goal -> "+selected_;labels.markers.push_back(marker);labels_pub_->publish(labels);
  }
  double scan_period_{.2};std::map<std::string,Clock::time_point> last_scan_;
  std::vector<std::string> robots_;std::string selected_;
  std::map<std::string,Robot> states_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::vector<rclcpp::PublisherBase::SharedPtr> publishers_;
  rclcpp::Publisher<TF>::SharedPtr tf_pub_,static_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr labels_pub_;
  std::map<std::string,geometry_msgs::msg::TransformStamped> static_cache_;
  rclcpp::Service<Trigger>::SharedPtr cancel_service_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  rclcpp::TimerBase::SharedPtr label_timer_,socket_timer_;
  int listen_fd_{-1};std::vector<Client> clients_;
};
std::shared_ptr<rclcpp::Node> make_fleet_node(bool with_socket) {
  return std::make_shared<FleetRuntime>(with_socket);
}
