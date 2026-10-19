#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace tb3_factor_graph_slam
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

struct ScanMatchResult
{
  bool converged{false};
  double fitness_score{0.0};
  // The refined transform that maps `source`'s local frame into
  // `target`'s local frame -- i.e. source's pose expressed in target's
  // frame. This is exactly the "measured" value a GTSAM BetweenFactor
  // expects between the two corresponding nodes.
  Pose2D relative_pose;
};

class ScanMatcher
{
public:
  // Converts a 2D laser scan into an XY point cloud (z = 0) in the
  // sensor's own local frame, skipping invalid/out-of-range readings.
  static pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(
    const sensor_msgs::msg::LaserScan & scan);

  // Aligns `source` onto `target` via point-to-point ICP, seeded with
  // `initial_guess` (typically an odometry- or pose-estimate-derived
  // relative pose). `converged` is false if PCL didn't converge OR the
  // final fitness score exceeds `fitness_threshold` -- callers should
  // treat either case as "don't trust this measurement".
  static ScanMatchResult align(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & target,
    const Pose2D & initial_guess,
    double max_correspondence_distance = 0.5,
    int max_iterations = 50,
    double fitness_threshold = 0.05);
};

}  // namespace tb3_factor_graph_slam