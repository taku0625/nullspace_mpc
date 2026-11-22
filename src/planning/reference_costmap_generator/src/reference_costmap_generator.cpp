#include "reference_costmap_generator/reference_costmap_generator.hpp"

#include <rclcpp/time.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace planning
{
grid_map::GridMap make_reference_costmap(
  const nav_msgs::msg::Path & path, double resolution, double distance_limit)
{
  if (!std::isfinite(resolution) || resolution <= 0.0 || !std::isfinite(distance_limit) ||
    distance_limit <= 0.0 || distance_limit > std::numeric_limits<float>::max())
  {
    throw std::invalid_argument("Reference costmap resolution and distance limit must be positive");
  }
  if (path.header.frame_id.empty() || path.poses.empty()) {
    throw std::invalid_argument("The reference costmap requires a path with a fixed frame and poses");
  }

  std::vector<grid_map::Position> points;
  points.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    const grid_map::Position point(pose.pose.position.x, pose.pose.position.y);
    if (!point.allFinite() ||
      (!pose.header.frame_id.empty() && pose.header.frame_id != path.header.frame_id))
    {
      throw std::invalid_argument("Reference path poses must be finite and in the path frame");
    }
    if (points.empty() || point != points.back()) {
      points.push_back(point);
    }
  }

  auto minimum = points.front();
  auto maximum = minimum;
  for (const auto & point : points) {
    minimum = minimum.cwiseMin(point);
    maximum = maximum.cwiseMax(point);
  }
  const grid_map::Length span = (maximum - minimum).array();
  const double margin = distance_limit + resolution;
  const grid_map::Length cells = ((span + 2.0 * margin) / resolution).ceil();
  // Bound memory use for the two layers and their serialized copies.
  if (!cells.allFinite() || cells.prod() > 8'000'000) {
    throw std::invalid_argument("The reference costmap is too large. Increase its resolution in meters.");
  }

  const float capped_distance = static_cast<float>(distance_limit);
  const float unknown_yaw = std::numeric_limits<float>::quiet_NaN();
  grid_map::GridMap map;
  map.setFrameId(path.header.frame_id);
  map.setTimestamp(rclcpp::Time(path.header.stamp).nanoseconds());
  map.setGeometry(cells * resolution, resolution, minimum + 0.5 * (maximum - minimum));
  map.add("distance_error", capped_distance);
  map.add("ref_yaw", unknown_yaw);
  map.setBasicLayers({"distance_error"});

  auto & distances = map["distance_error"];
  auto & headings = map["ref_yaw"];
  grid_map::Position first_cell;
  map.getPosition(grid_map::Index::Zero(), first_cell);

  // Visit the cells within reach of each segment. Distances are exact within the cap.
  const auto rasterize = [&](const grid_map::Position & a, const grid_map::Position & b) {
      const grid_map::Vector direction = b - a;
      const double length_squared = direction.squaredNorm();
      const float yaw = length_squared > 0.0 ?
        static_cast<float>(std::atan2(direction.y(), direction.x())) : unknown_yaw;
      const Eigen::Array2d low = a.cwiseMin(b).array() - distance_limit;
      const Eigen::Array2d high = a.cwiseMax(b).array() + distance_limit;
      const grid_map::Index begin = ((first_cell.array() - high) / resolution).ceil()
        .max(0.0).cast<int>();
      const grid_map::Index end = ((first_cell.array() - low) / resolution).floor()
        .min(map.getSize().cast<double>() - 1.0).cast<int>();
      for (int y = begin.y(); y <= end.y(); ++y) {
        const double cell_y = first_cell.y() - y * resolution;
        for (int x = begin.x(); x <= end.x(); ++x) {
          const grid_map::Vector offset(first_cell.x() - x * resolution - a.x(), cell_y - a.y());
          const double along = length_squared > 0.0 ?
            std::clamp(offset.dot(direction) / length_squared, 0.0, 1.0) : 0.0;
          const float distance = static_cast<float>((offset - along * direction).norm());
          if (distance < distances(x, y)) {
            distances(x, y) = distance;
            headings(x, y) = yaw;
          }
        }
      }
    };

  if (points.size() == 1) {
    rasterize(points.front(), points.front());
  } else {
    for (std::size_t i = 1; i < points.size(); ++i) {
      rasterize(points[i - 1], points[i]);
    }
  }
  return map;
}
}  // namespace planning
