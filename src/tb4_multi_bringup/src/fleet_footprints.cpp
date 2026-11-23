// Native port of MARCO's fleet_obstacles.py (MACRO/src/simulations/ros2) for the real fleet.
// For every robot it republishes the shared map with the *other* robots' round
// footprints stamped as occupied, on /<robot>/fleet_map. Nav2 costmaps (static
// layer) read that map, so planners and controllers keep clear of peers even
// when the peer is outside the LiDAR view. The reference map remains clean.
// It also draws every robot's footprint in RViz (/tb4_view/fleet_footprints).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using Grid = nav_msgs::msg::OccupancyGrid;
using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class FleetFootprints : public rclcpp::Node {
public:
  FleetFootprints() : Node("fleet_footprints") {
    std::stringstream ids(declare_parameter<std::string>("robot_ids", "robot1,robot2"));
    for (std::string id; std::getline(ids, id, ',');) if (!id.empty()) robots_.push_back(id);
    body_radius_ = declare_parameter<double>("body_radius", .171);   // Create 3 base
    peer_margin_ = declare_parameter<double>("peer_margin", .01);    // MARCO PEER_MARGIN
    min_move_ = declare_parameter<double>("min_move", .01);
    const double rate = declare_parameter<double>("rate", 5.);
    const auto map_topic = declare_parameter<std::string>("map_topic", "/" + (robots_.empty() ? std::string("robot1") : robots_.front()) + "/map");
    if (robots_.size() < 1 || robots_.size() > 3 || !(body_radius_ > .05 && body_radius_ < .5) ||
        !(peer_margin_ >= 0. && peer_margin_ < .3) || !(min_move_ > 0.) || !(rate > 0. && rate <= 20.))
      throw std::invalid_argument("Invalid fleet footprint parameters");
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    map_sub_ = create_subscription<Grid>(map_topic, latched, [this](Grid::ConstSharedPtr m) {
      map_ = m; for (auto& entry : published_) entry.second.clear(); });
    for (const auto& r : robots_) {
      pose_subs_.push_back(create_subscription<Pose>("/" + r + "/pose", 10,
        [this, r](Pose::ConstSharedPtr m) {
          if (m->header.frame_id == "map" && std::isfinite(m->pose.pose.position.x) && std::isfinite(m->pose.pose.position.y))
            poses_[r] = {m->pose.pose.position.x, m->pose.pose.position.y};
        }));
      map_pubs_[r] = create_publisher<Grid>("/" + r + "/fleet_map", latched);
    }
    markers_ = create_publisher<MarkerArray>("/tb4_view/fleet_footprints", 10);
    timer_ = create_wall_timer(std::chrono::duration<double>(1. / rate), [this] { tick(); });
    RCLCPP_INFO(get_logger(), "Peer footprints r=%.3f m (+%.3f margin) for %zu robot(s) from %s",
                body_radius_, peer_margin_, robots_.size(), map_topic.c_str());
  }

private:
  struct Point { double x, y; };

  void tick() {
    if (map_) for (const auto& own : robots_) publish_map(own);
    publish_markers();
  }

  // Republish only when a peer moved: each new map makes the costmap static layer copy it.
  void publish_map(const std::string& own) {
    std::vector<std::optional<Point>> peers;
    for (const auto& other : robots_) {
      if (other == own) continue;
      const auto it = poses_.find(other);
      peers.push_back(it == poses_.end() ? std::nullopt : std::optional<Point>(it->second));
    }
    auto& last = published_[own];
    bool changed = last.size() != peers.size();
    for (size_t i = 0; !changed && i < peers.size(); ++i)
      changed = peers[i].has_value() != last[i].has_value() ||
                (peers[i] && std::hypot(peers[i]->x - last[i]->x, peers[i]->y - last[i]->y) >= min_move_);
    if (!changed) return;
    last = peers;
    Grid grid = *map_;
    grid.header.stamp = now();
    const auto& info = grid.info;
    const double res = info.resolution, oyaw = 2. * std::atan2(info.origin.orientation.z, info.origin.orientation.w);
    const double c = std::cos(oyaw), s = std::sin(oyaw), radius = body_radius_ + peer_margin_;
    const double reach = radius + res / 2.;   // half-cell raster padding encloses boundary cells
    const int w = int(info.width), h = int(info.height);
    for (const auto& peer : peers) {
      if (!peer) continue;
      const double dx0 = peer->x - info.origin.position.x, dy0 = peer->y - info.origin.position.y;
      const double gx = (c * dx0 + s * dy0) / res, gy = (-s * dx0 + c * dy0) / res;
      const int span = int(std::ceil(reach / res)) + 1;
      for (int row = std::max(0, int(gy) - span); row <= std::min(h - 1, int(gy) + span); ++row)
        for (int col = std::max(0, int(gx) - span); col <= std::min(w - 1, int(gx) + span); ++col) {
          const double ex = (col + .5 - gx) * res, ey = (row + .5 - gy) * res;
          if (ex * ex + ey * ey <= reach * reach) grid.data[size_t(row) * w + col] = 100;
        }
    }
    map_pubs_.at(own)->publish(grid);
  }

  void publish_markers() {
    static const double colors[3][3] = {{.2, .8, 1.}, {1., .55, .1}, {.3, .9, .3}};
    MarkerArray array;
    for (size_t i = 0; i < robots_.size(); ++i) {
      const auto it = poses_.find(robots_[i]);
      if (it == poses_.end()) continue;
      Marker m; m.header.frame_id = "map"; m.header.stamp = now();
      m.ns = "fleet_footprints"; m.id = int(i); m.type = Marker::LINE_STRIP; m.action = Marker::ADD;
      m.pose.orientation.w = 1.; m.scale.x = .02; m.lifetime = rclcpp::Duration::from_seconds(1.);
      m.color.r = colors[i % 3][0]; m.color.g = colors[i % 3][1]; m.color.b = colors[i % 3][2]; m.color.a = 1.;
      const double radius = body_radius_ + peer_margin_;
      for (int k = 0; k <= 32; ++k) {
        geometry_msgs::msg::Point p;
        p.x = it->second.x + radius * std::cos(2. * M_PI * k / 32); p.y = it->second.y + radius * std::sin(2. * M_PI * k / 32); p.z = .03;
        m.points.push_back(p);
      }
      array.markers.push_back(m);
    }
    if (!array.markers.empty()) markers_->publish(array);
  }

  std::vector<std::string> robots_;
  double body_radius_{.171}, peer_margin_{.01}, min_move_{.01};
  Grid::ConstSharedPtr map_;
  std::map<std::string, Point> poses_;
  std::map<std::string, std::vector<std::optional<Point>>> published_;
  std::map<std::string, rclcpp::Publisher<Grid>::SharedPtr> map_pubs_;
  std::vector<rclcpp::Subscription<Pose>::SharedPtr> pose_subs_;
  rclcpp::Subscription<Grid>::SharedPtr map_sub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr markers_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FleetFootprints>());
  rclcpp::shutdown();
  return 0;
}
