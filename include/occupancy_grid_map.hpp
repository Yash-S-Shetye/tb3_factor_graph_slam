#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "scan_matcher.hpp"  // for Pose2D

namespace tb3_factor_graph_slam
{

// A raw LiDAR scan kept in the sensor's own frame, so it can be re-projected
// into the map every time the keyframe's pose estimate changes (e.g. after
// a loop closure moves the whole trajectory).
struct StoredScan
{
  std::vector<float> ranges;
  float angle_min{0.0f};
  float angle_increment{0.0f};
  float range_min{0.0f};
  float range_max{0.0f};

  static StoredScan fromLaserScan(const sensor_msgs::msg::LaserScan & scan);
};

// Log-odds occupancy grid. Each scan is integrated by ray casting:
// cells a beam passes through get "more free", the cell where a beam ends
// gets "more occupied". Evidence accumulates by simple addition, which is
// why log-odds is the standard representation.
class OccupancyGridMap
{
public:
  OccupancyGridMap(
    double width_m, double height_m, double resolution,
    double origin_x, double origin_y);

  // Forget everything (all cells back to "unknown").
  void reset();

  // Integrate one scan taken from `sensor_pose` (the LiDAR's pose in the
  // map frame, NOT the robot base pose).
  void integrateScan(const Pose2D & sensor_pose, const StoredScan & scan);

  // Convert to a ROS message. Unknown cells are -1, known cells are
  // 0..100 (occupancy probability). `x_offset` shifts the published
  // origin along x without moving the map's contents in the grid, which
  // lets a second map be displayed beside the first for comparison.
  nav_msgs::msg::OccupancyGrid toMsg(
    const std::string & frame_id, const rclcpp::Time & stamp,
    double x_offset = 0.0) const;

private:
  void traceRay(int x0, int y0, int x1, int y1, bool hit);
  void update(int ix, int iy, float delta);

  int toIndexX(double x) const;
  int toIndexY(double y) const;

  int width_cells_;
  int height_cells_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  std::vector<float> log_odds_;

  static constexpr float kFreeDelta = -0.4f;
  static constexpr float kOccupiedDelta = 0.85f;
  static constexpr float kMinLogOdds = -5.0f;
  static constexpr float kMaxLogOdds = 5.0f;
  // Safety cap so a bogus range_max can't make a single ray enormous.
  static constexpr double kMaxRayRange = 10.0;
};

}  // namespace tb3_factor_graph_slam