#include "reference_costmap_generator/reference_costmap_generator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace planning
{
namespace
{
nav_msgs::msg::Path make_path(std::initializer_list<grid_map::Position> points)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp.sec = 14;
  for (const auto & point : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = point.x();
    pose.pose.position.y = point.y();
    path.poses.push_back(pose);
  }
  return path;
}

TEST(ReferenceCostmap, StoresSegmentDistanceAndDirectionInThePathFrame)
{
  auto path = make_path({{-1.0, 1.0}, {2.0, 1.0}, {2.0, 1.0}, {2.0, 3.0}});
  const auto map = make_reference_costmap(path, 0.05, 0.60);
  EXPECT_EQ(map.getFrameId(), "map");
  EXPECT_EQ(map.getTimestamp(), 14'000'000'000ULL);
  EXPECT_DOUBLE_EQ(map.getResolution(), 0.05);

  grid_map::Index cell;
  grid_map::Position center;
  ASSERT_TRUE(map.getIndex({0.0, 1.2}, cell));
  ASSERT_TRUE(map.getPosition(cell, center));
  EXPECT_NEAR(map.at("distance_error", cell), std::abs(center.y() - 1.0), 1e-6);
  EXPECT_FLOAT_EQ(map.at("ref_yaw", cell), 0.0F);

  ASSERT_TRUE(map.getIndex({2.15, 2.0}, cell));
  ASSERT_TRUE(map.getPosition(cell, center));
  EXPECT_NEAR(map.at("distance_error", cell), std::abs(center.x() - 2.0), 1e-6);
  EXPECT_NEAR(map.at("ref_yaw", cell), std::atan2(1.0, 0.0), 1e-6);

  ASSERT_TRUE(map.getIndex({-1.3, 2.8}, cell));
  EXPECT_FLOAT_EQ(map.at("distance_error", cell), 0.60F);
  EXPECT_TRUE(std::isnan(map.at("ref_yaw", cell)));
}

TEST(ReferenceCostmap, HandlesAStationaryPathAndRejectsInvalidInputs)
{
  auto path = make_path({{2.0, -1.0}, {2.0, -1.0}});
  const auto map = make_reference_costmap(path, 0.10, 0.60);
  grid_map::Index cell;
  ASSERT_TRUE(map.getIndex({2.0, -1.0}, cell));
  EXPECT_LT(map.at("distance_error", cell), 0.08F);
  EXPECT_TRUE(std::isnan(map.at("ref_yaw", cell)));

  path.poses[1].header.frame_id = "odom";
  EXPECT_THROW(make_reference_costmap(path, 0.1, 0.6), std::invalid_argument);
  EXPECT_THROW(make_reference_costmap(make_path({}), 0.1, 0.6), std::invalid_argument);
  EXPECT_THROW(make_reference_costmap(make_path({{0.0, 0.0}, {1000.0, 1000.0}}),
    0.001, 0.6), std::invalid_argument);
}
}  // namespace
}  // namespace planning
