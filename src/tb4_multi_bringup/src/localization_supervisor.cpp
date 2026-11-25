// PC-only readiness gate for AMCL, EKF and Nav2.
#include <map>
#include <sstream>
#include "scan_hit.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2/buffer_core.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
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
    has_hint_=declare_parameter<bool>("initial_pose_known",false);
    hint_x_=declare_parameter<double>("hint_x",0.);
    hint_y_=declare_parameter<double>("hint_y",0.);
    hint_std_xy_=declare_parameter<double>("hint_std_xy",.2);
    base_frame_=declare_parameter<std::string>("base_frame","base_link");
    map_frame_=declare_parameter<std::string>("map_frame","map");
    if(!std::isfinite(hint_x_)||!std::isfinite(hint_y_)||!std::isfinite(hint_std_xy_)||
       hint_std_xy_<.05||hint_std_xy_>1.)throw std::invalid_argument("Invalid localization hint");
    navigate_=declare_parameter<bool>("navigation",true);
    const auto profile=declare_parameter<std::string>("localization_profile","strict");
    if(profile!="strict"&&profile!="navigation"&&profile!="demo")throw std::invalid_argument("Unknown localization profile");
    const bool interactive=profile!="strict";
    // "demo" (video recording): ride out multi-second Wi-Fi stalls and loose AMCL
    // instead of cancelling paths; MARCO's swept-contact trigger stays active.
    const bool demo=profile=="demo";
    // "navigation" (MARCO on the real fleet) is deliberately tolerant: office
    // Wi-Fi delays scans and AMCL settles slowly; "strict" stays for tests.
    max_covariance_xy_=demo?.20:interactive?.09:.01;
    max_covariance_yaw_=demo?.40:interactive?.20:.04;
    min_scan_match_=demo?.35:interactive?.50:.70;
    recover_covariance_xy_=demo?.30:interactive?.15:.04;
    max_scan_age_s_=demo?3.0:interactive?1.0:.5;
    max_scan_gap_=std::chrono::milliseconds(demo?3500:interactive?1200:700);
    max_odom_gap_=std::chrono::milliseconds(demo?3000:interactive?1000:500);
    ready_hold_=interactive?std::chrono::milliseconds(1000):std::chrono::milliseconds(3000);
    pause_after_=std::chrono::milliseconds(demo?10000:interactive?5000:2000);
    nav_=create_client<Manage>("lifecycle_manager_navigation/manage_nodes");
    nomotion_=create_client<Empty>("request_nomotion_update");
    status_pub_=create_publisher<std_msgs::msg::String>("localization_status",rclcpp::QoS(1).transient_local());
    const auto live=rclcpp::SensorDataQoS();
    const auto latched=rclcpp::QoS(1).transient_local();
    hint_sub_=create_subscription<Pose>("initialpose",10,[this](Pose::ConstSharedPtr m){
      const auto& p=m->pose.pose.position;
      if(m->header.frame_id!=map_frame_||!std::isfinite(p.x)||!std::isfinite(p.y))return;
      has_hint_=true;hint_verified_=false;hint_x_=p.x;hint_y_=p.y;
      reseed_at_=now();quality_started_=false;pose_.reset();
    });
    pose_sub_=create_subscription<Pose>("pose",latched,[this](Pose::ConstSharedPtr m){
      if(m->header.frame_id==map_frame_&&rclcpp::Time(m->header.stamp)>=reseed_at_)pose_=m;
    });
    // Peer robots are real but unmapped obstacles: scan returns on their
    // bodies must not count against map agreement (MARCO peer footprints).
    {
      std::stringstream ids(declare_parameter<std::string>("peer_ids",""));
      peer_radius_=declare_parameter<double>("peer_radius",.25);
      if(!(peer_radius_>0.&&peer_radius_<.6))throw std::invalid_argument("peer_radius must be within 0..0.6 m");
      for(std::string id;std::getline(ids,id,',');) if(!id.empty())
        peer_subs_.push_back(create_subscription<Pose>("/"+id+"/pose",10,[this,id](Pose::ConstSharedPtr m){
          if(m->header.frame_id==map_frame_)peers_[id]={m->pose.pose.position.x,m->pose.pose.position.y,Clock::now()};}));
    }
    scan_sub_=create_subscription<sensor_msgs::msg::LaserScan>("scan",live,[this](sensor_msgs::msg::LaserScan::ConstSharedPtr m){scan_=m;scan_at_=Clock::now();});
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("navigation/wheel_odom",live,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr){odom_at_=Clock::now();});
    filtered_sub_=create_subscription<nav_msgs::msg::Odometry>("odometry/filtered",live,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m){
        if(m->child_frame_id==base_frame_)filtered_at_=Clock::now();
      });
    imu_sub_=create_subscription<sensor_msgs::msg::Imu>("navigation/imu",live,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr){imu_at_=Clock::now();});
    map_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("map",latched,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){if(valid_map(*m))map_=m;else map_.reset();});
    tf_sub_=create_subscription<tf2_msgs::msg::TFMessage>("navigation/tf",rclcpp::QoS(100).best_effort(),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m){transforms(*m,false);});
    static_sub_=create_subscription<tf2_msgs::msg::TFMessage>("navigation/tf_static",latched,
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m){transforms(*m,true);});
    timer_=create_wall_timer(200ms,[this]{tick();});
  }
private:
  bool valid_map(const nav_msgs::msg::OccupancyGrid& m) const {
    const auto size=uint64_t(m.info.width)*m.info.height;
    return m.header.frame_id==map_frame_&&size>0&&size<=4000000&&m.data.size()==size&&
           std::isfinite(m.info.resolution)&&m.info.resolution>0.;
  }
  void transforms(const tf2_msgs::msg::TFMessage& m,bool is_static) {
    for(const auto& t:m.transforms)try{buffer_.setTransform(t,"navigation",is_static);}catch(const tf2::TransformException&){}
  }
  bool quality(Json& status) {
    if(!map_){status["reason"]="waiting_for_reference_map";return false;}
    if(!has_hint_){status["reason"]="waiting_for_initial_pose";return false;}
    if(!scan_){status["reason"]="waiting_for_scan";return false;}
    const double scan_age=now().seconds()-scan_->header.stamp.sec-scan_->header.stamp.nanosec*1e-9;
    const auto received=Clock::now();
    const auto scan_gap=received-scan_at_;
    const auto odom_gap=received-odom_at_;
    status["scan_stamp_age_s"]=scan_age;
    status["scan_receipt_age_s"]=std::chrono::duration<double>(scan_gap).count();
    status["odom_receipt_age_s"]=std::chrono::duration<double>(odom_gap).count();
    status["ekf_receipt_age_s"]=std::chrono::duration<double>(received-filtered_at_).count();
    status["imu_receipt_age_s"]=std::chrono::duration<double>(received-imu_at_).count();
    if(scan_age<-.1||scan_age>max_scan_age_s_){status["reason"]="stale_scan_timestamp";return false;}
    if(scan_gap>max_scan_gap_||odom_gap>max_odom_gap_){status["reason"]="stale_sensors";return false;}
    if(received-filtered_at_>max_odom_gap_||received-imu_at_>max_odom_gap_){status["reason"]="stale_fusion";return false;}
    if(!pose_){status["reason"]="waiting_for_amcl_pose";return false;}
    const double pose_age=(now()-rclcpp::Time(pose_->header.stamp)).seconds();
    status["amcl_pose_age_s"]=pose_age;
    if(pose_age<-.1||pose_age>2.){status["reason"]="stale_amcl_pose";return false;}
    try {
      const auto body=buffer_.lookupTransform(map_frame_,base_frame_,tf2::TimePointZero);
      const double age=now().seconds()-body.header.stamp.sec-body.header.stamp.nanosec*1e-9;
      status["tf_age_s"]=age;
      if(age<-.1||age>.5){status["reason"]="stale_map_tf";return false;}
      const auto laser=buffer_.lookupTransform(map_frame_,scan_->header.frame_id,tf2::TimePointZero).transform;
      const double a=yaw(laser.rotation),oa=yaw(map_->info.origin.orientation),co=std::cos(oa),si=std::sin(oa),res=map_->info.resolution;
      if(!std::isfinite(res)||res<=0)return false;
      std::vector<std::pair<double,double>> peers;
      for(const auto& [id,p]:peers_) if(Clock::now()-p.at<2s)peers.emplace_back(p.x,p.y);
      int hits=0,total=0,peer_points=0;const int stride=std::max(1,int(scan_->ranges.size()/180));
      for(size_t i=0;i<scan_->ranges.size();i+=stride) {
        const double r=scan_->ranges[i];if(!std::isfinite(r)||r<scan_->range_min||r>=scan_->range_max)continue;
        const double angle=scan_->angle_min+i*scan_->angle_increment+a;
        const double dx=laser.translation.x+r*std::cos(angle)-map_->info.origin.position.x;
        const double dy=laser.translation.y+r*std::sin(angle)-map_->info.origin.position.y;
        const double mx=laser.translation.x+r*std::cos(angle),my=laser.translation.y+r*std::sin(angle);
        bool on_peer=false;
        for(const auto& p:peers) if(std::hypot(mx-p.first,my-p.second)<=peer_radius_){on_peer=true;break;}
        if(on_peer){++peer_points;continue;}
        const int x=std::floor((co*dx+si*dy)/res),y=std::floor((-si*dx+co*dy)/res);++total;
        if(tb4::near_occupied_cell(map_->data,int(map_->info.width),int(map_->info.height),x,y,res,.15))++hits;
      }
      if(has_hint_&&!hint_verified_&&std::hypot(body.transform.translation.x-hint_x_,body.transform.translation.y-hint_y_)>std::max(.15,2*hint_std_xy_)){status["reason"]="outside_initial_hint_region";return false;}
      const double score=total?double(hits)/total:0;
      status["scan_match_fraction"]=score;
      if(peer_points)status["scan_peer_points"]=peer_points;
      const auto& cov=pose_->pose.covariance;
      status["covariance_xyyaw"]={cov[0],cov[7],cov[35]};
      status["pose"]={body.transform.translation.x,body.transform.translation.y,yaw(body.transform.rotation)};
      // Accept only numerical roundoff; materially negative variances still fail.
      const bool ok=total>=20&&score>=min_scan_match_&&std::isfinite(cov[0])&&std::isfinite(cov[7])&&std::isfinite(cov[35])&&cov[0]>=-1e-10&&cov[7]>=-1e-10&&cov[35]>=-1e-10&&cov[0]<=max_covariance_xy_&&cov[7]<=max_covariance_xy_&&cov[35]<=max_covariance_yaw_;
      status["reason"]=ok?"stable_localization":"amcl_not_converged";return ok;
    }catch(const tf2::TransformException&){status["reason"]="waiting_for_map_tf";return false;}
  }
  void manage(uint8_t command,const std::string& next) {
    auto req=std::make_shared<Manage::Request>();req->command=command;
    pending_=true;pending_at_=Clock::now();
    if(command==Manage::Request::STARTUP)phase_="navigation";
    nav_->async_send_request(req,[this,next](rclcpp::Client<Manage>::SharedFuture f){
      if(!pending_)return;
      pending_=false;
      try {phase_=f.get()->success?next:"nav_lifecycle_failed";}
      catch(const std::exception&){phase_="nav_lifecycle_failed";}
    });
  }
  void refresh_amcl(const Json& status, Clock::time_point stamp) {
    // AMCL may keep publishing TF while stationary without updating its pose.
    // Ask it to process a new scan before the pose freshness limit expires.
    if (!has_hint_ || !map_ || stamp-last_update_<1s ||
        stamp-scan_at_>700ms || stamp-odom_at_>500ms ||
        stamp-filtered_at_>500ms || stamp-imu_at_>500ms) return;
    const auto reason=status.value("reason",std::string());
    if (pose_ && status.value("amcl_pose_age_s",0.)<1.0 &&
        reason!="amcl_not_converged" && reason!="outside_initial_hint_region") return;
    if (!nomotion_->service_is_ready()) return;
    nomotion_->remove_pending_request(update_request_id_);
    update_request_id_=nomotion_->async_send_request(std::make_shared<Empty::Request>()).request_id;
    last_update_=stamp;
  }
  void tick() {
    Json status={{"ready",false},{"pose_ready",false},{"recovering",false}};
    const auto stamp=Clock::now();
    if(pending_&&stamp-pending_at_>65s){phase_="nav_lifecycle_timeout";pending_=false;}
    const bool ok=quality(status);
    refresh_amcl(status, stamp);
    bool recovering=false;
    if(!ok&&status.value("reason",std::string())=="amcl_not_converged"&&
       status.contains("covariance_xyyaw")&&status.value("scan_match_fraction",0.)>=min_scan_match_) {
      const auto& c=status["covariance_xyyaw"];
      recovering=c[0].get<double>()>=0.&&c[0].get<double>()<=recover_covariance_xy_&&
                 c[1].get<double>()>=0.&&c[1].get<double>()<=recover_covariance_xy_&&
                 c[2].get<double>()>=0.&&c[2].get<double>()<=.09;
    }
    if(!ok){if(!bad_quality_)bad_since_=stamp;bad_quality_=true;}else bad_quality_=false;
    if(ok){if(!quality_started_)quality_since_=stamp;quality_started_=true;}else quality_started_=false;
    const bool ready=ok&&quality_started_&&((phase_=="running"&&hint_verified_)||stamp-quality_since_>=ready_hold_);
    if(ready)hint_verified_=true;
    if(!pending_&&phase_=="localizing"&&ready) {
      if(!navigate_)phase_="running";
      else if(nav_->service_is_ready())manage(Manage::Request::STARTUP,"running");
    }
    // Nav2 bringup can abort on a transient service timeout while several
    // robots' stacks start at once (lifecycle manager: "Aborting bringup").
    // Reset the navigation lifecycle and start it again instead of staying failed.
    if(phase_=="nav_lifecycle_failed"||phase_=="nav_lifecycle_timeout") {
      if(!nav_failed_seen_){nav_failed_seen_=true;nav_failed_at_=stamp;}
      if(!pending_&&navigate_&&nav_->service_is_ready()&&nav_retries_<5&&stamp-nav_failed_at_>=5s) {
        ++nav_retries_;nav_failed_seen_=false;
        RCLCPP_WARN(get_logger(),"Navigation lifecycle bringup failed; reset and retry %d/5",nav_retries_);
        manage(Manage::Request::RESET,"localizing");
      }
    } else nav_failed_seen_=false;
    if(!pending_&&navigate_&&nav_->service_is_ready()) {
      if(phase_=="running"&&bad_quality_&&stamp-bad_since_>(recovering?std::chrono::milliseconds(10000):pause_after_))
        manage(Manage::Request::PAUSE,"paused");
      else if(phase_=="paused"&&ready)manage(Manage::Request::RESUME,"running");
    }
    status["ready"]=ready&&phase_=="running"&&!pending_;
    status["pose_ready"]=ready;
    status["recovering"]=recovering&&phase_=="running"&&!pending_;
    if(ready&&phase_=="navigation")status["reason"]="waiting_for_nav2_startup";
    if(phase_=="nav_lifecycle_failed"||phase_=="nav_lifecycle_timeout")status["reason"]=phase_;
    status["phase"]=phase_;
    std_msgs::msg::String message;message.data=status.dump();status_pub_->publish(message);
    if(last_phase_!=phase_){RCLCPP_INFO(get_logger(),"Localization state: %s",phase_.c_str());last_phase_=phase_;}
  }

  struct Peer{double x,y;Clock::time_point at;};
  std::map<std::string,Peer> peers_;std::vector<rclcpp::Subscription<Pose>::SharedPtr> peer_subs_;
  bool has_hint_{false},hint_verified_{false};
  bool navigate_{true},pending_{false},quality_started_{false},bad_quality_{false};
  bool nav_failed_seen_{false};int nav_retries_{0};Clock::time_point nav_failed_at_{};
  double max_scan_age_s_{.5};std::chrono::milliseconds max_scan_gap_{700},max_odom_gap_{500},ready_hold_{3000},pause_after_{2000};
  double hint_x_{0},hint_y_{0},hint_std_xy_{.2},peer_radius_{.25};
  double max_covariance_xy_{.01},max_covariance_yaw_{.04},min_scan_match_{.70},recover_covariance_xy_{.04};
  std::string phase_{"localizing"},last_phase_,map_frame_,base_frame_;
  Clock::time_point pending_at_{},scan_at_{},odom_at_{},filtered_at_{},imu_at_{},quality_since_{},bad_since_{};
  Clock::time_point last_update_{};
  int64_t update_request_id_{0};
  rclcpp::Time reseed_at_{0,0,RCL_ROS_TIME};
  tf2::BufferCore buffer_;
  Pose::ConstSharedPtr pose_;sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  rclcpp::Client<Manage>::SharedPtr nav_;
  rclcpp::Client<Empty>::SharedPtr nomotion_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<Pose>::SharedPtr pose_sub_,hint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_,filtered_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_,static_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){try{rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<LocalizationSupervisor>());}catch(const std::exception& e){RCLCPP_ERROR(rclcpp::get_logger("localization_supervisor"),"%s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
