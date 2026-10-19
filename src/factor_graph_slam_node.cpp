// factor_graph_slam_node.cpp
//
// Project 2: 2D pose graph SLAM with ICP scan matching and GTSAM.
//
// Builds on project 1's keyframe/edge scaffolding, with two additions:
//   1. Odometry edges are refined by ICP between consecutive scans
//      instead of being trusted raw from wheel odometry.
//   2. When the robot revisits a location, a loop-closure edge is added
//      and the whole GTSAM factor graph is re-optimized (batch LM),
//      pulling the drifted trajectory back into consistency.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"

#include "scan_matcher.hpp"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

using std::placeholders::_1;
using tb3_factor_graph_slam::Pose2D;
using tb3_factor_graph_slam::ScanMatcher;
using tb3_factor_graph_slam::ScanMatchResult;

namespace
{

double yawFromQuaternion(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double normalizeAngle(double angle)
{
  while (angle > M_PI) {angle -= 2.0 * M_PI;}
  while (angle < -M_PI) {angle += 2.0 * M_PI;}
  return angle;
}

// a (+) b: places b (given in a's local frame) into the global frame a
// is expressed in. Standard SE(2) pose composition.
Pose2D compose(const Pose2D & a, const Pose2D & b)
{
  Pose2D out;
  const double c = std::cos(a.theta);
  const double s = std::sin(a.theta);
  out.x = a.x + c * b.x - s * b.y;
  out.y = a.y + s * b.x + c * b.y;
  out.theta = normalizeAngle(a.theta + b.theta);
  return out;
}

// b expressed in a's local frame (i.e. the "rel" such that
// compose(a, rel) == b). Matches GTSAM's Pose2::between semantics.
Pose2D relativeTransform(const Pose2D & a, const Pose2D & b)
{
  Pose2D out;
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double c = std::cos(-a.theta);
  const double s = std::sin(-a.theta);
  out.x = c * dx - s * dy;
  out.y = s * dx + c * dy;
  out.theta = normalizeAngle(b.theta - a.theta);
  return out;
}

}  // namespace

struct Keyframe
{
  int id{0};
  Pose2D raw_odom_pose;  // straight from /odom -- diagnostic only, never fed to GTSAM
  Pose2D estimate;       // this node's current best pose in the graph's own frame
  Pose2D gt_pose;
  bool has_gt{false};
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

struct GraphEdgeInfo
{
  int from_id;
  int to_id;
  bool is_loop_closure;
};

class FactorGraphSlamNode : public rclcpp::Node
{
public:
  FactorGraphSlamNode()
  : Node("factor_graph_slam_node")
  {
    declare_parameter<std::string>("odom_topic", "/odom");
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("ground_truth_topic", "/ground_truth/state");
    declare_parameter<double>("keyframe_linear_threshold", 0.3);
    declare_parameter<double>("keyframe_angular_threshold", 0.3);
    declare_parameter<double>("loop_closure_search_radius", 1.0);
    declare_parameter<int>("loop_closure_min_index_gap", 20);
    declare_parameter<double>("loop_closure_fitness_threshold", 0.05);
    declare_parameter<double>("icp_fitness_threshold", 0.08);

    odom_topic_ = get_parameter("odom_topic").as_string();
    scan_topic_ = get_parameter("scan_topic").as_string();
    ground_truth_topic_ = get_parameter("ground_truth_topic").as_string();
    linear_threshold_ = get_parameter("keyframe_linear_threshold").as_double();
    angular_threshold_ = get_parameter("keyframe_angular_threshold").as_double();
    loop_search_radius_ = get_parameter("loop_closure_search_radius").as_double();
    loop_min_index_gap_ = get_parameter("loop_closure_min_index_gap").as_int();
    loop_fitness_threshold_ = get_parameter("loop_closure_fitness_threshold").as_double();
    icp_fitness_threshold_ = get_parameter("icp_fitness_threshold").as_double();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&FactorGraphSlamNode::odomCallback, this, _1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FactorGraphSlamNode::scanCallback, this, _1));
    ground_truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      ground_truth_topic_, 10,
      std::bind(&FactorGraphSlamNode::groundTruthCallback, this, _1));

    raw_odom_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/factor_graph/raw_odom_path", 10);
    optimized_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/factor_graph/optimized_path", 10);
    gt_path_pub_ = create_publisher<nav_msgs::msg::Path>("/factor_graph/gt_path", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/factor_graph/graph_markers", 10);
    raw_drift_pub_ = create_publisher<std_msgs::msg::Float64>("/factor_graph/raw_drift", 10);
    optimized_drift_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/factor_graph/optimized_drift", 10);
    loop_closure_count_pub_ = create_publisher<std_msgs::msg::Int32>(
      "/factor_graph/loop_closure_count", 10);

    raw_odom_path_.header.frame_id = "odom";
    optimized_path_.header.frame_id = "odom";
    gt_path_.header.frame_id = "odom";

    RCLCPP_INFO(
      get_logger(),
      "factor_graph_slam_node started. Keyframe thresholds: %.2f m / %.2f rad. "
      "Loop closure search radius: %.2f m (min index gap %d).",
      linear_threshold_, angular_threshold_, loop_search_radius_, loop_min_index_gap_);
  }

private:
  void groundTruthCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;
    latest_gt_pose_.x = p.x;
    latest_gt_pose_.y = p.y;
    latest_gt_pose_.theta = yawFromQuaternion(q.x, q.y, q.z, q.w);
    have_gt_ = true;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    latest_scan_ = msg;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!latest_scan_) {
      return;  // wait for the first scan before starting the graph
    }

    Pose2D current;
    current.x = msg->pose.pose.position.x;
    current.y = msg->pose.pose.position.y;
    const auto & q = msg->pose.pose.orientation;
    current.theta = yawFromQuaternion(q.x, q.y, q.z, q.w);

    if (keyframes_.empty()) {
      addFirstKeyframe(current, msg->header.stamp);
      return;
    }

    const Pose2D & last_raw = keyframes_.back().raw_odom_pose;
    const double dist = std::hypot(current.x - last_raw.x, current.y - last_raw.y);
    const double dtheta = std::fabs(normalizeAngle(current.theta - last_raw.theta));

    if (dist >= linear_threshold_ || dtheta >= angular_threshold_) {
      addKeyframe(current, msg->header.stamp);
    }
  }

  void addFirstKeyframe(const Pose2D & raw_odom_pose, const rclcpp::Time & stamp)
  {
    Keyframe kf;
    kf.id = 0;
    kf.raw_odom_pose = raw_odom_pose;
    // Anchor node 0 at the first odometry pose (not at 0,0,0) so the whole
    // graph lives in the same frame as /odom and the ground truth.
    kf.estimate = raw_odom_pose;
    if (have_gt_) {
      kf.gt_pose = latest_gt_pose_;
      kf.has_gt = true;
    }
    kf.cloud = ScanMatcher::toPointCloud(*latest_scan_);
    keyframes_.push_back(kf);

    const auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector3(1e-3, 1e-3, 1e-4));
    const gtsam::Pose2 anchor(raw_odom_pose.x, raw_odom_pose.y, raw_odom_pose.theta);
    graph_.addPrior(gtsam::Symbol('x', 0), anchor, prior_noise);
    initial_values_.insert(gtsam::Symbol('x', 0), anchor);

    publishAll(stamp);
  }

  void addKeyframe(const Pose2D & raw_odom_pose, const rclcpp::Time & stamp)
  {
    const Keyframe & prev = keyframes_.back();
    const Pose2D odom_relative = relativeTransform(prev.raw_odom_pose, raw_odom_pose);

    auto current_cloud = ScanMatcher::toPointCloud(*latest_scan_);
    const ScanMatchResult icp_result = ScanMatcher::align(
      current_cloud, prev.cloud, odom_relative,
      /*max_correspondence_distance=*/0.5, /*max_iterations=*/50, icp_fitness_threshold_);

    Pose2D edge_pose = odom_relative;
    gtsam::SharedNoiseModel edge_noise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector3(0.1, 0.1, 0.05));  // fallback: trust raw odometry loosely

    if (icp_result.converged) {
      edge_pose = icp_result.relative_pose;
      edge_noise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(0.03, 0.03, 0.01));  // scan match succeeded: trust it more
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Scan match to previous keyframe did not converge (fitness=%.3f); "
        "falling back to raw odometry for this edge.",
        icp_result.fitness_score);
    }

    Keyframe kf;
    kf.id = static_cast<int>(keyframes_.size());
    kf.raw_odom_pose = raw_odom_pose;
    kf.estimate = compose(prev.estimate, edge_pose);
    if (have_gt_) {
      kf.gt_pose = latest_gt_pose_;
      kf.has_gt = true;
    }
    kf.cloud = current_cloud;

    graph_.add(
      gtsam::BetweenFactor<gtsam::Pose2>(
        gtsam::Symbol('x', prev.id), gtsam::Symbol('x', kf.id),
        gtsam::Pose2(edge_pose.x, edge_pose.y, edge_pose.theta), edge_noise));
    initial_values_.insert(
      gtsam::Symbol('x', kf.id),
      gtsam::Pose2(kf.estimate.x, kf.estimate.y, kf.estimate.theta));
    edge_info_.push_back({prev.id, kf.id, false});

    keyframes_.push_back(kf);

    const bool loop_closed = tryLoopClosure(keyframes_.back());
    if (loop_closed) {
      optimizeGraph();
    }

    publishAll(stamp);
  }

  bool tryLoopClosure(Keyframe & new_kf)
  {
    int best_candidate = -1;
    double best_dist = loop_search_radius_;

    for (const auto & kf : keyframes_) {
      if (new_kf.id - kf.id < loop_min_index_gap_) {
        continue;  // too recent to count as a genuine "revisit"
      }
      const double d = std::hypot(
        new_kf.estimate.x - kf.estimate.x, new_kf.estimate.y - kf.estimate.y);
      if (d < best_dist) {
        best_dist = d;
        best_candidate = kf.id;
      }
    }

    if (best_candidate < 0) {
      return false;
    }

    const Keyframe & candidate = keyframes_[best_candidate];
    const Pose2D initial_guess = relativeTransform(candidate.estimate, new_kf.estimate);

    const ScanMatchResult icp_result = ScanMatcher::align(
      new_kf.cloud, candidate.cloud, initial_guess,
      /*max_correspondence_distance=*/0.5, /*max_iterations=*/60, loop_fitness_threshold_);

    if (!icp_result.converged) {
      return false;
    }

    const auto loop_noise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector3(0.05, 0.05, 0.02));
    graph_.add(
      gtsam::BetweenFactor<gtsam::Pose2>(
        gtsam::Symbol('x', candidate.id), gtsam::Symbol('x', new_kf.id),
        gtsam::Pose2(
          icp_result.relative_pose.x, icp_result.relative_pose.y,
          icp_result.relative_pose.theta),
        loop_noise));
    edge_info_.push_back({candidate.id, new_kf.id, true});
    loop_closure_count_++;

    RCLCPP_INFO(
      get_logger(),
      "Loop closure: node %d <-> node %d (dist=%.2f m, fitness=%.4f). "
      "Total closures: %d",
      candidate.id, new_kf.id, best_dist, icp_result.fitness_score, loop_closure_count_);

    return true;
  }

  void optimizeGraph()
  {
    gtsam::LevenbergMarquardtOptimizer optimizer(graph_, initial_values_);
    const gtsam::Values result = optimizer.optimize();
    initial_values_ = result;  // warm-start the next optimization

    for (auto & kf : keyframes_) {
      const gtsam::Pose2 p = result.at<gtsam::Pose2>(gtsam::Symbol('x', kf.id));
      kf.estimate = Pose2D{p.x(), p.y(), p.theta()};
    }
  }

  void publishAll(const rclcpp::Time & stamp)
  {
    publishPaths(stamp);
    publishMarkers();
    publishMetrics();
  }

  void publishPaths(const rclcpp::Time & stamp)
  {
    raw_odom_path_.header.stamp = stamp;
    optimized_path_.header.stamp = stamp;
    gt_path_.header.stamp = stamp;
    raw_odom_path_.poses.clear();
    optimized_path_.poses.clear();
    gt_path_.poses.clear();

    auto toPoseStamped = [&](const Pose2D & pose) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "odom";
      ps.header.stamp = stamp;
      ps.pose.position.x = pose.x;
      ps.pose.position.y = pose.y;
      ps.pose.orientation.z = std::sin(pose.theta / 2.0);
      ps.pose.orientation.w = std::cos(pose.theta / 2.0);
      return ps;
    };

    for (const auto & kf : keyframes_) {
      raw_odom_path_.poses.push_back(toPoseStamped(kf.raw_odom_pose));
      optimized_path_.poses.push_back(toPoseStamped(kf.estimate));
      if (kf.has_gt) {
        gt_path_.poses.push_back(toPoseStamped(kf.gt_pose));
      }
    }

    raw_odom_path_pub_->publish(raw_odom_path_);
    optimized_path_pub_->publish(optimized_path_);
    if (!gt_path_.poses.empty()) {
      gt_path_pub_->publish(gt_path_);
    }
  }

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray array;
    const auto stamp = now();

    visualization_msgs::msg::Marker node_marker;
    node_marker.header.frame_id = "odom";
    node_marker.header.stamp = stamp;
    node_marker.ns = "nodes";
    node_marker.id = 0;
    node_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    node_marker.action = visualization_msgs::msg::Marker::ADD;
    node_marker.scale.x = node_marker.scale.y = node_marker.scale.z = 0.08;
    node_marker.color.r = 0.1f;
    node_marker.color.g = 0.6f;
    node_marker.color.b = 1.0f;
    node_marker.color.a = 1.0f;

    visualization_msgs::msg::Marker odom_edge_marker;
    odom_edge_marker.header = node_marker.header;
    odom_edge_marker.ns = "odom_edges";
    odom_edge_marker.id = 1;
    odom_edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    odom_edge_marker.action = visualization_msgs::msg::Marker::ADD;
    odom_edge_marker.scale.x = 0.02;
    odom_edge_marker.color.r = 1.0f;
    odom_edge_marker.color.g = 0.6f;
    odom_edge_marker.color.b = 0.0f;
    odom_edge_marker.color.a = 1.0f;

    visualization_msgs::msg::Marker loop_edge_marker;
    loop_edge_marker.header = node_marker.header;
    loop_edge_marker.ns = "loop_edges";
    loop_edge_marker.id = 2;
    loop_edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    loop_edge_marker.action = visualization_msgs::msg::Marker::ADD;
    loop_edge_marker.scale.x = 0.03;
    loop_edge_marker.color.r = 1.0f;
    loop_edge_marker.color.g = 0.0f;
    loop_edge_marker.color.b = 0.0f;
    loop_edge_marker.color.a = 1.0f;

    for (const auto & kf : keyframes_) {
      geometry_msgs::msg::Point pt;
      pt.x = kf.estimate.x;
      pt.y = kf.estimate.y;
      node_marker.points.push_back(pt);
    }

    auto pointFor = [&](int id) {
      geometry_msgs::msg::Point pt;
      pt.x = keyframes_[id].estimate.x;
      pt.y = keyframes_[id].estimate.y;
      return pt;
    };

    for (const auto & edge : edge_info_) {
      auto & marker = edge.is_loop_closure ? loop_edge_marker : odom_edge_marker;
      marker.points.push_back(pointFor(edge.from_id));
      marker.points.push_back(pointFor(edge.to_id));
    }

    array.markers.push_back(node_marker);
    array.markers.push_back(odom_edge_marker);
    array.markers.push_back(loop_edge_marker);
    marker_pub_->publish(array);
  }

  void publishMetrics()
  {
    if (keyframes_.empty() || !keyframes_.back().has_gt) {
      return;
    }
    const auto & kf = keyframes_.back();

    std_msgs::msg::Float64 raw_msg;
    raw_msg.data = std::hypot(
      kf.raw_odom_pose.x - kf.gt_pose.x, kf.raw_odom_pose.y - kf.gt_pose.y);
    raw_drift_pub_->publish(raw_msg);

    std_msgs::msg::Float64 opt_msg;
    opt_msg.data = std::hypot(kf.estimate.x - kf.gt_pose.x, kf.estimate.y - kf.gt_pose.y);
    optimized_drift_pub_->publish(opt_msg);

    std_msgs::msg::Int32 count_msg;
    count_msg.data = loop_closure_count_;
    loop_closure_count_pub_->publish(count_msg);
  }

  // parameters
  std::string odom_topic_, scan_topic_, ground_truth_topic_;
  double linear_threshold_, angular_threshold_;
  double loop_search_radius_;
  int loop_min_index_gap_;
  double loop_fitness_threshold_;
  double icp_fitness_threshold_;

  // state
  std::vector<Keyframe> keyframes_;
  std::vector<GraphEdgeInfo> edge_info_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  Pose2D latest_gt_pose_;
  bool have_gt_{false};
  int loop_closure_count_{0};

  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_values_;

  nav_msgs::msg::Path raw_odom_path_, optimized_path_, gt_path_;

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
    raw_odom_path_pub_, optimized_path_pub_, gt_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr raw_drift_pub_, optimized_drift_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr loop_closure_count_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FactorGraphSlamNode>());
  rclcpp::shutdown();
  return 0;
}