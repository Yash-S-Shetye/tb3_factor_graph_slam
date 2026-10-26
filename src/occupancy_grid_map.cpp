#include "occupancy_grid_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace tb3_factor_graph_slam
{

StoredScan StoredScan::fromLaserScan(const sensor_msgs::msg::LaserScan & scan)
{
  StoredScan out;
  out.ranges = scan.ranges;
  out.angle_min = scan.angle_min;
  out.angle_increment = scan.angle_increment;
  out.range_min = scan.range_min;
  out.range_max = scan.range_max;
  return out;
}

OccupancyGridMap::OccupancyGridMap(
  double width_m, double height_m, double resolution,
  double origin_x, double origin_y)
: width_cells_(static_cast<int>(std::ceil(width_m / resolution))),
  height_cells_(static_cast<int>(std::ceil(height_m / resolution))),
  resolution_(resolution),
  origin_x_(origin_x),
  origin_y_(origin_y),
  log_odds_(static_cast<size_t>(width_cells_) * static_cast<size_t>(height_cells_), 0.0f)
{
}

void OccupancyGridMap::reset()
{
  std::fill(log_odds_.begin(), log_odds_.end(), 0.0f);
}

int OccupancyGridMap::toIndexX(double x) const
{
  return static_cast<int>(std::floor((x - origin_x_) / resolution_));
}

int OccupancyGridMap::toIndexY(double y) const
{
  return static_cast<int>(std::floor((y - origin_y_) / resolution_));
}

void OccupancyGridMap::update(int ix, int iy, float delta)
{
  if (ix < 0 || iy < 0 || ix >= width_cells_ || iy >= height_cells_) {
    return;  // outside the grid: ignore rather than crash
  }
  float & cell = log_odds_[static_cast<size_t>(iy) * width_cells_ + ix];
  cell = std::clamp(cell + delta, kMinLogOdds, kMaxLogOdds);
}

// Bresenham line from the sensor cell to the beam's end cell. Every cell
// before the end is free space; the end cell is occupied only if the beam
// actually hit something (as opposed to reaching max range).
void OccupancyGridMap::traceRay(int x0, int y0, int x1, int y1, bool hit)
{
  const int dx = std::abs(x1 - x0);
  const int dy = -std::abs(y1 - y0);
  const int sx = x0 < x1 ? 1 : -1;
  const int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  int x = x0;
  int y = y0;

  while (true) {
    if (x == x1 && y == y1) {
      update(x, y, hit ? kOccupiedDelta : kFreeDelta);
      break;
    }
    update(x, y, kFreeDelta);
    const int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y += sy;
    }
  }
}

void OccupancyGridMap::integrateScan(const Pose2D & sensor_pose, const StoredScan & scan)
{
  const int sensor_ix = toIndexX(sensor_pose.x);
  const int sensor_iy = toIndexY(sensor_pose.y);

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const float r = scan.ranges[i];
    if (std::isnan(r) || r < scan.range_min) {
      continue;  // invalid reading
    }

    double effective_range;
    bool hit;
    if (std::isinf(r) || r >= scan.range_max) {
      // No return: the beam travelled its full length through free space.
      effective_range = std::min(static_cast<double>(scan.range_max), kMaxRayRange);
      hit = false;
    } else {
      effective_range = std::min(static_cast<double>(r), kMaxRayRange);
      hit = (static_cast<double>(r) <= kMaxRayRange);
    }

    const double angle =
      sensor_pose.theta + scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double end_x = sensor_pose.x + effective_range * std::cos(angle);
    const double end_y = sensor_pose.y + effective_range * std::sin(angle);

    traceRay(sensor_ix, sensor_iy, toIndexX(end_x), toIndexY(end_y), hit);
  }
}

nav_msgs::msg::OccupancyGrid OccupancyGridMap::toMsg(
  const std::string & frame_id, const rclcpp::Time & stamp, double x_offset) const
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.info.map_load_time = stamp;
  msg.info.resolution = static_cast<float>(resolution_);
  msg.info.width = static_cast<uint32_t>(width_cells_);
  msg.info.height = static_cast<uint32_t>(height_cells_);
  msg.info.origin.position.x = origin_x_ + x_offset;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.data.resize(log_odds_.size());
  for (size_t i = 0; i < log_odds_.size(); ++i) {
    const float l = log_odds_[i];
    if (l == 0.0f) {
      msg.data[i] = -1;  // never observed
    } else {
      const double p = 1.0 - 1.0 / (1.0 + std::exp(static_cast<double>(l)));
      msg.data[i] = static_cast<int8_t>(std::lround(100.0 * p));
    }
  }
  return msg;
}

}  // namespace tb3_factor_graph_slam