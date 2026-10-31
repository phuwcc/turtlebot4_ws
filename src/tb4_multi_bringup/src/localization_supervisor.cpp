// PC-only lifecycle/AMCL initialization. Never publishes motor commands.
#include "global_scan_match.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2/buffer_core.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
using namespace std::chrono_literals;
using Clock=std::chrono::steady_clock;
using Manage=nav2_msgs::srv::ManageLifecycleNodes;
using Empty=std_srvs::srv::Empty;
using Pose=geometry_msgs::msg::PoseWithCovarianceStamped;
using Json=nlohmann::json;
static double yaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
}
class LocalizationSupervisor:public rclcpp::Node {
public:
  LocalizationSupervisor():Node("localization_supervisor") {
    anchor_=declare_parameter<bool>("anchor",false);
    has_hint_=declare_parameter<bool>("hint",false);
    hint_x_=declare_parameter<double>("hint_x",0.);hint_y_=declare_parameter<double>("hint_y",0.);hint_yaw_=declare_parameter<double>("hint_yaw",0.);
    hint_std_xy_=declare_parameter<double>("hint_std_xy",.2);hint_std_yaw_=declare_parameter<double>("hint_std_yaw",.2);
    if(!std::isfinite(hint_x_)||!std::isfinite(hint_y_)||!std::isfinite(hint_yaw_)||!std::isfinite(hint_std_xy_)||!std::isfinite(hint_std_yaw_)||hint_std_xy_<.05||hint_std_xy_>1.||hint_std_yaw_<.05||hint_std_yaw_>M_PI)throw std::invalid_argument("Invalid localization hint");
    anchor_yaw_=declare_parameter<double>("anchor_yaw",0.0);
    navigate_=declare_parameter<bool>("navigation",true);
    wait_seconds_=declare_parameter<double>("global_timeout",120.0);
    if(!std::isfinite(anchor_yaw_)||!std::isfinite(wait_seconds_)||wait_seconds_<=0)throw std::invalid_argument("Invalid anchor yaw or global timeout");
    loc_=create_client<Manage>("lifecycle_manager_localization/manage_nodes");
    nav_=create_client<Manage>("lifecycle_manager_navigation/manage_nodes");
    global_=create_client<Empty>("reinitialize_global_localization");
    update_=create_client<Empty>("request_nomotion_update");
    pose_pub_=create_publisher<Pose>("initialpose",10);
    status_pub_=create_publisher<std_msgs::msg::String>("localization_status",rclcpp::QoS(1).transient_local());
    const auto live=rclcpp::SensorDataQoS();
    hint_sub_=create_subscription<Pose>("initialpose",10,[this](Pose::ConstSharedPtr){phase_at_=Clock::now();quality_started_=false;});
    pose_sub_=create_subscription<Pose>("amcl_pose",rclcpp::QoS(1).transient_local(),[this](Pose::ConstSharedPtr m){pose_=m;});
    scan_sub_=create_subscription<sensor_msgs::msg::LaserScan>("scan",live,[this](sensor_msgs::msg::LaserScan::ConstSharedPtr m){scan_=m;scan_at_=Clock::now();});
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("odom",live,[this](nav_msgs::msg::Odometry::ConstSharedPtr){odom_at_=Clock::now();});
    map_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("map",rclcpp::QoS(1).transient_local(),[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){map_=m;distance_field();});
    tf_sub_=create_subscription<tf2_msgs::msg::TFMessage>("tf",rclcpp::QoS(100).best_effort(),[this](tf2_msgs::msg::TFMessage::ConstSharedPtr m){transforms(*m,false);});
    static_sub_=create_subscription<tf2_msgs::msg::TFMessage>("tf_static",rclcpp::QoS(100).transient_local(),[this](tf2_msgs::msg::TFMessage::ConstSharedPtr m){transforms(*m,true);});
    began_=Clock::now();timer_=create_wall_timer(200ms,[this]{tick();});
  }
private:
  void transforms(const tf2_msgs::msg::TFMessage& m,bool is_static) {
    for(const auto& t:m.transforms)try{buffer_.setTransform(t,"hardware",is_static);}catch(const tf2::TransformException&){}
  }
  void distance_field() {
    const int w=map_->info.width,h=map_->info.height;
    if(w<=0||h<=0||uint64_t(map_->info.width)*map_->info.height>4000000||map_->data.size()!=size_t(w)*h){dist_.clear();return;}
    dist_.assign(w*h,1000000);std::deque<int> queue;
    for(int i=0;i<w*h;++i)if(map_->data[i]>=65){dist_[i]=0;queue.push_back(i);}
    while(!queue.empty()) {
      const int i=queue.front();queue.pop_front();const int x=i%w,y=i/w;
      for(const int j:{x>0?i-1:-1,x+1<w?i+1:-1,y>0?i-w:-1,y+1<h?i+w:-1})
        if(j>=0&&dist_[j]>dist_[i]+1){dist_[j]=dist_[i]+1;queue.push_back(j);}
    }
  }
  bool quality(Json& status) {
    if(!pose_||!map_||!scan_||dist_.empty()){status["reason"]="waiting_for_map_scan_amcl";return false;}
    const double scan_age=now().seconds()-scan_->header.stamp.sec-scan_->header.stamp.nanosec*1e-9;
    if(scan_age<-.1||scan_age>.5){status["reason"]="stale_scan_timestamp";return false;}
    if(Clock::now()-scan_at_>700ms||Clock::now()-odom_at_>500ms){status["reason"]="stale_sensors";return false;}
    try {
      const auto body=buffer_.lookupTransform("map","base_link",tf2::TimePointZero);
      const double age=now().seconds()-body.header.stamp.sec-body.header.stamp.nanosec*1e-9;
      status["tf_age_s"]=age;
      if(age<-.1||age>.5){status["reason"]="stale_map_tf";return false;}
      const auto laser=buffer_.lookupTransform("map",scan_->header.frame_id,tf2::TimePointZero).transform;
      const double a=yaw(laser.rotation),oa=yaw(map_->info.origin.orientation),co=std::cos(oa),si=std::sin(oa),res=map_->info.resolution;
      if(!std::isfinite(res)||res<=0)return false;
      int hits=0,total=0;const int stride=std::max(1,int(scan_->ranges.size()/180));
      for(size_t i=0;i<scan_->ranges.size();i+=stride) {
        const double r=scan_->ranges[i];if(!std::isfinite(r)||r<scan_->range_min||r>=scan_->range_max)continue;
        const double angle=scan_->angle_min+i*scan_->angle_increment+a;
        const double dx=laser.translation.x+r*std::cos(angle)-map_->info.origin.position.x;
        const double dy=laser.translation.y+r*std::sin(angle)-map_->info.origin.position.y;
        const int x=std::floor((co*dx+si*dy)/res),y=std::floor((-si*dx+co*dy)/res);++total;
        if(x>=0&&y>=0&&x<int(map_->info.width)&&y<int(map_->info.height)&&dist_[y*map_->info.width+x]*res<=.15)++hits;
      }
      if(has_hint_&&!hint_verified_&&std::hypot(body.transform.translation.x-hint_x_,body.transform.translation.y-hint_y_)>std::max(.15,2*hint_std_xy_)){status["reason"]="outside_initial_hint_region";return false;}
      const double score=total?double(hits)/total:0;
      status["scan_match_fraction"]=score;
      const auto& cov=pose_->pose.covariance;
      status["covariance_xyyaw"]={cov[0],cov[7],cov[35]};
      status["pose"]={body.transform.translation.x,body.transform.translation.y,yaw(body.transform.rotation)};
      // AMCL may return -1e-14 from floating-point cancellation after resampling.
      // Accept only numerical roundoff; materially negative variances still fail.
      const bool ok=total>=20&&score>=.70&&std::isfinite(cov[0])&&std::isfinite(cov[7])&&std::isfinite(cov[35])&&cov[0]>=-1e-10&&cov[7]>=-1e-10&&cov[35]>=-1e-10&&cov[0]<=.01&&cov[7]<=.01&&cov[35]<=.04;
      status["reason"]=ok?"stable_localization":"amcl_not_converged";return ok;
    }catch(const tf2::TransformException&){status["reason"]="waiting_for_map_tf";return false;}
  }
  void manage(rclcpp::Client<Manage>::SharedPtr client,uint8_t command,const std::string& next) {
    auto req=std::make_shared<Manage::Request>();req->command=command;pending_=true;pending_at_=Clock::now();const auto generation=++generation_;const bool localization=client==loc_;
    if(!localization&&command==Manage::Request::STARTUP)phase_="navigation";
    client->async_send_request(req,[this,next,generation,localization](rclcpp::Client<Manage>::SharedFuture f){
      if(!pending_||generation!=generation_)return;
      pending_=false;
      try {if(f.get()->success){phase_=next;phase_at_=Clock::now();}else{phase_=localization?(next=="discovery"?"reset_failed":"lifecycle_failed"):"nav_lifecycle_failed";}}
      catch(const std::exception&){phase_=localization?"lifecycle_failed":"nav_lifecycle_failed";}
    });
  }
  void initialize() {
    if(anchor_||has_hint_) {
      if(pose_pub_->get_subscription_count()<2)return;
      Pose m;m.header.frame_id="map";m.header.stamp=now();const double a=anchor_?anchor_yaw_:hint_yaw_;m.pose.pose.position.x=anchor_?0.:hint_x_;m.pose.pose.position.y=anchor_?0.:hint_y_;m.pose.pose.orientation.z=std::sin(a/2);m.pose.pose.orientation.w=std::cos(a/2);
      m.pose.covariance[0]=m.pose.covariance[7]=anchor_?.0004:hint_std_xy_*hint_std_xy_;m.pose.covariance[35]=anchor_?.0025:hint_std_yaw_*hint_std_yaw_;pose_pub_->publish(m);
      RCLCPP_INFO(get_logger(),"%s initialized at map (%.3f,%.3f), yaw %.3f; AMCL validation required",anchor_?"Robot1 anchor":"Initial hint",m.pose.pose.position.x,m.pose.pose.position.y,a);
      phase_="localizing";phase_at_=Clock::now();
    }else if(global_->service_is_ready()) {
      pending_=true;pending_at_=Clock::now();const auto generation=++generation_;
      global_->async_send_request(std::make_shared<Empty::Request>(),[this,generation](rclcpp::Client<Empty>::SharedFuture f){
        if(!pending_||generation_!=generation)return;
        pending_=false;try{f.get();phase_="localizing";phase_at_=Clock::now();RCLCPP_INFO(get_logger(),"Global AMCL search started; waiting for convergence, no movement requested");}catch(const std::exception&){phase_="initialization_failed";}
      });
    }
  }
  void assist_global_search() {
    if(anchor_||has_hint_||!map_||!scan_||dist_.empty()||Clock::now()-phase_at_<10s||Clock::now()-last_search_<20s)return;
    last_search_=Clock::now();
    try {
      const auto sensor=buffer_.lookupTransform("base_link",scan_->header.frame_id,tf2::TimePointZero).transform;
      const double angle=yaw(sensor.rotation);std::vector<tb4::ScanPoint> points;
      const size_t stride=std::max(size_t(1),scan_->ranges.size()/72);
      for(size_t i=0;i<scan_->ranges.size();i+=stride) {
        const double r=scan_->ranges[i];if(!std::isfinite(r)||r<scan_->range_min||r>=scan_->range_max)continue;
        const double a=angle+scan_->angle_min+i*scan_->angle_increment;
        points.push_back({sensor.translation.x+r*std::cos(a),sensor.translation.y+r*std::sin(a)});
      }
      const auto best=tb4::global_scan_match(map_->info.width,map_->info.height,map_->info.resolution,map_->data,dist_,points);
      search_audit_={{"score",best.score},{"mode_gap",best.gap},{"unique",best.unique}};
      if(!best.unique||assisted_)return;
      const double ox=best.x*map_->info.resolution,oy=best.y*map_->info.resolution,oa=yaw(map_->info.origin.orientation);
      Pose seed;seed.header.frame_id="map";seed.header.stamp=now();
      seed.pose.pose.position.x=map_->info.origin.position.x+std::cos(oa)*ox-std::sin(oa)*oy;
      seed.pose.pose.position.y=map_->info.origin.position.y+std::sin(oa)*ox+std::cos(oa)*oy;
      const double a=best.yaw+oa;seed.pose.pose.orientation.z=std::sin(a/2);seed.pose.pose.orientation.w=std::cos(a/2);
      seed.pose.covariance[0]=seed.pose.covariance[7]=.04;seed.pose.covariance[35]=.04;
      pose_pub_->publish(seed);assisted_=true;quality_started_=false;
      RCLCPP_INFO(get_logger(),"Unique scan-map seed (score %.3f, gap %.3f) sent to AMCL; convergence still required",best.score,best.gap);
    }catch(const tf2::TransformException&){}
  }
  void tick() {
    Json status={{"phase",phase_},{"ready",false},{"anchor",anchor_}};
    if(pending_&&Clock::now()-pending_at_>65s){phase_="lifecycle_timeout";pending_=false;++generation_;}
    if(!pending_) {
      if(phase_=="discovery"&&Clock::now()-began_>5s&&loc_->service_is_ready()) {
        ++attempts_;manage(loc_,Manage::Request::STARTUP,"initialize");
      }else if(phase_=="lifecycle_failed"&&attempts_<2&&loc_->service_is_ready()) {
        manage(loc_,Manage::Request::RESET,"discovery");began_=Clock::now();
      }else if(phase_=="initialize"&&map_&&scan_&&Clock::now()-odom_at_<500ms) initialize();
    }
    if(phase_=="localizing"||phase_=="navigation"||phase_=="running"||phase_=="paused") {
      const bool ok=quality(status);
      if(!ok){if(!bad_quality_)bad_since_=Clock::now();bad_quality_=true;}else bad_quality_=false;
      if(!ok&&phase_=="localizing")assist_global_search();
      if(!search_audit_.is_null())status["global_scan_search"]=search_audit_;
      if(ok) {if(!quality_started_)quality_since_=Clock::now();quality_started_=true;}
      else quality_started_=false;
      const bool ready=ok&&quality_started_&&Clock::now()-quality_since_>=3s;
      if(ready)hint_verified_=true; // A seed constrains initialization, not later navigation.
      status["pose_ready"]=ready;
      status["ready"]=ready&&(!navigate_||phase_=="running");
      if(ready&&phase_=="navigation")status["reason"]="waiting_for_nav2_startup";
      if(phase_=="localizing"&&!pending_&&ready) {
        if(navigate_&&nav_->service_is_ready())manage(nav_,Manage::Request::STARTUP,"running");
        else if(!navigate_)phase_="running";
      }
      if(phase_=="running"&&navigate_&&bad_quality_&&Clock::now()-bad_since_>2s&&!pending_&&nav_->service_is_ready())manage(nav_,Manage::Request::PAUSE,"paused");
      if(phase_=="paused"&&ready&&!pending_&&nav_->service_is_ready())manage(nav_,Manage::Request::RESUME,"running");
      if(phase_=="localizing"&&Clock::now()-phase_at_>std::chrono::duration<double>(wait_seconds_))status["reason"]=has_hint_?"hint_not_converged_check_scan_alignment":"global_search_ambiguous_set_manual_initial_pose";
      if(!ready&&Clock::now()-last_update_>1s&&Clock::now()-phase_at_<std::chrono::duration<double>(wait_seconds_)&&update_->service_is_ready()) {
        last_update_=Clock::now();update_->async_send_request(std::make_shared<Empty::Request>());
      }
    }
    status["phase"]=phase_;std_msgs::msg::String message;message.data=status.dump();status_pub_->publish(message);
    if(last_phase_!=phase_){RCLCPP_INFO(get_logger(),"Localization state: %s",phase_.c_str());last_phase_=phase_;}
  }
  bool has_hint_{false},hint_verified_{false};double hint_x_{0},hint_y_{0},hint_yaw_{0},hint_std_xy_{.2},hint_std_yaw_{.2};
  bool bad_quality_{false};Clock::time_point bad_since_;
  Json search_audit_=nullptr;bool assisted_{false};Clock::time_point last_search_;
  uint64_t generation_{0};
  bool anchor_{false},navigate_{true},pending_{false},quality_started_{false};int attempts_{0};double anchor_yaw_{0},wait_seconds_{120};
  std::string phase_{"discovery"},last_phase_;Clock::time_point began_,phase_at_,pending_at_,scan_at_,odom_at_,quality_since_,last_update_;
  tf2::BufferCore buffer_;std::vector<int> dist_;
  Pose::ConstSharedPtr pose_;sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  rclcpp::Client<Manage>::SharedPtr loc_,nav_;rclcpp::Client<Empty>::SharedPtr global_,update_;
  rclcpp::Publisher<Pose>::SharedPtr pose_pub_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<Pose>::SharedPtr pose_sub_,hint_sub_;rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_,static_sub_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){try{rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<LocalizationSupervisor>());}catch(const std::exception& e){RCLCPP_ERROR(rclcpp::get_logger("localization_supervisor"),"%s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
