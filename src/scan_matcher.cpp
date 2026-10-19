#include "scan_matcher.hpp"

#include <cmath>

#include <Eigen/Dense>
#include <pcl/registration/icp.h>

namespace tb3_factor_graph_slam
{

pcl::PointCloud<pcl::PointXYZ>::Ptr ScanMatcher::toPointCloud(
  const sensor_msgs::msg::LaserScan & scan)
{
  auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->reserve(scan.ranges.size());

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const float range = scan.ranges[i];
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      continue;
    }
    const float angle = scan.angle_min + static_cast<float>(i) * scan.angle_increment;
    pcl::PointXYZ pt;
    pt.x = range * std::cos(angle);
    pt.y = range * std::sin(angle);
    pt.z = 0.0f;
    cloud->push_back(pt);
  }

  return cloud;
}

namespace
{

Eigen::Matrix4f toEigen(const Pose2D & pose)
{
  Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
  const float c = static_cast<float>(std::cos(pose.theta));
  const float s = static_cast<float>(std::sin(pose.theta));
  mat(0, 0) = c;
  mat(0, 1) = -s;
  mat(0, 3) = static_cast<float>(pose.x);
  mat(1, 0) = s;
  mat(1, 1) = c;
  mat(1, 3) = static_cast<float>(pose.y);
  return mat;
}

Pose2D fromEigen(const Eigen::Matrix4f & mat)
{
  Pose2D pose;
  pose.x = mat(0, 3);
  pose.y = mat(1, 3);
  pose.theta = std::atan2(mat(1, 0), mat(0, 0));
  return pose;
}

}  // namespace

ScanMatchResult ScanMatcher::align(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & source,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & target,
  const Pose2D & initial_guess,
  double max_correspondence_distance,
  int max_iterations,
  double fitness_threshold)
{
  ScanMatchResult result;

  // ICP needs enough points on both sides to have a well-posed problem;
  // an empty or near-empty scan (e.g. right at startup) can't be matched.
  if (source->size() < 10 || target->size() < 10) {
    return result;
  }

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(source);
  icp.setInputTarget(target);
  icp.setMaxCorrespondenceDistance(max_correspondence_distance);
  icp.setMaximumIterations(max_iterations);

  pcl::PointCloud<pcl::PointXYZ> aligned;
  icp.align(aligned, toEigen(initial_guess));

  result.converged = icp.hasConverged();
  result.fitness_score = icp.getFitnessScore();
  result.relative_pose = fromEigen(icp.getFinalTransformation());

  if (result.fitness_score > fitness_threshold) {
    result.converged = false;
  }

  return result;
}

}  // namespace tb3_factor_graph_slam