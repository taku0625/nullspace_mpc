#include "ros_inputs.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <mpc_eval_msgs/msg/mpc_eval.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/create_timer.hpp>
#include <rviz_2d_overlay_msgs/msg/overlay_text.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <array>
#include <chrono>
#include <string>
#include <utility>

namespace mppi_unicycle
{
class SimulationController : public rclcpp::Node
{
public:
  SimulationController()
  : Node("mppi_unicycle"),
    settings_(declare_settings(*this)), core_(settings_),
    odom_timeout_(positive_parameter(*this, "odom_timeout", 0.4)),
    tire_radius_(positive_parameter(*this, "tire_radius", 0.2)),
    front_(positive_parameter(*this, "l_f", 0.5)), rear_(positive_parameter(*this, "l_r", 0.5)),
    left_(positive_parameter(*this, "d_l", 0.5)), right_(positive_parameter(*this, "d_r", 0.5)),
    tf_(get_clock()), listener_(tf_, this, false)
  {
    command_ = create_publisher<geometry_msgs::msg::Twist>(topic("control_cmd_vel_topic", "/cmd_vel"), 1);
    eval_ = create_publisher<mpc_eval_msgs::msg::MPCEval>("/mpc/eval_info", 1);
    optimal_ = create_publisher<visualization_msgs::msg::MarkerArray>("/mpc/optimal_traj", 1);
    sampled_ = create_publisher<visualization_msgs::msg::MarkerArray>("/mpc/sampled_traj", 1);
    overlay_ = create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>("/mpc/overlay_text", 1);
    const std::array<std::string, 5> names{
      "/mpc/cmd/absvel", "/mpc/cmd/vx", "/mpc/cmd/vy", "/mpc/cmd/omega", "/mpc/calc_time",
    };
    for (std::size_t i = 0; i < names.size(); ++i) {
      telemetry_[i] = create_publisher<std_msgs::msg::Float32>(names[i], 1);
    }

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      topic("odom_topic", "/groundtruth_odom"), rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr message) {
        odom_ = message;
        received_ = now();
      });
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      topic("ref_path_topic", "/plan"), 1,
      [this](const nav_msgs::msg::Path & message) {
        auto route = path_poses(message);
        if (route) {
          frame_ = message.header.frame_id;
          core_.set_route(std::move(*route));
        } else {
          frame_.clear();
          core_.set_route({});
          command_->publish(geometry_msgs::msg::Twist{});
        }
      });
    collision_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      topic("collision_costmap_topic", "/local_costmap/costmap"), 1,
      [this](const nav_msgs::msg::OccupancyGrid & grid) {
        collision_ready_ = grid_map::GridMapRosConverter::fromOccupancyGrid(
          grid, "collision_cost", collision_);
      });
    distance_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
      topic("distance_error_map_topic", "/distance_error_map"), 1,
      [this](const grid_map_msgs::msg::GridMap & grid) {
        distance_ready_ = grid_map::GridMapRosConverter::fromMessage(grid, distance_) &&
          distance_.exists("distance_error");
      });
    yaw_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
      topic("ref_yaw_map_topic", "/ref_yaw_map"), 1,
      [this](const grid_map_msgs::msg::GridMap & grid) {
        yaw_ready_ = grid_map::GridMapRosConverter::fromMessage(grid, yaw_) && yaw_.exists("ref_yaw");
      });
    const auto interval = positive_parameter(*this, "control_interval", 0.05);
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(interval),
      [this] {step();});
  }

private:
  std::string topic(const std::string & parameter, const std::string & fallback)
  {
    return declare_parameter(parameter, fallback);
  }

  std::optional<Pose> current_pose()
  {
    if (!odom_ || !received_ || odom_->header.frame_id.empty()) {
      return std::nullopt;
    }
    const double age = (now() - *received_).seconds();
    if (age < 0.0 || age > odom_timeout_) {
      return std::nullopt;
    }
    if (odom_->header.frame_id == frame_) {
      return planar_pose(odom_->pose.pose);
    }
    try {
      geometry_msgs::msg::PoseStamped from, to;
      from.header = odom_->header;
      from.pose = odom_->pose.pose;
      tf2::doTransform(from, to, tf_.lookupTransform(frame_, from.header.frame_id, tf2::TimePointZero));
      return planar_pose(to.pose);
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  MapCost map_cost(const Pose & pose) const
  {
    const grid_map::Position position(pose.x, pose.y);
    auto read_layer = [&position](const grid_map::GridMap & map, const char * layer)
      -> std::optional<double>
      {
        if (map.isInside(position)) {
          const double value = map.atPosition(layer, position);
          if (std::isfinite(value)) {
            return value;
          }
        }
        return std::nullopt;
      };
    return {collision_at(collision_, position), read_layer(distance_, "distance_error"),
      read_layer(yaw_, "ref_yaw")};
  }

  void step()
  {
    if (frame_.empty()) {
      return;
    }
    const auto pose = current_pose();
    if (!pose || !collision_ready_ || !distance_ready_ || !yaw_ready_ ||
      collision_.getFrameId() != frame_ || distance_.getFrameId() != frame_ ||
      yaw_.getFrameId() != frame_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Waiting for odometry and costmaps in the path frame");
      command_->publish(geometry_msgs::msg::Twist{});
      return;
    }
    const VOmega measured{odom_->twist.twist.linear.x, odom_->twist.twist.angular.z};
    const auto started = std::chrono::steady_clock::now();
    const auto result = core_.solve(*pose, measured, [this](const Pose & point) {
        return map_cost(point);
      }, sampled_->get_subscription_count() > 0 ? 16 : 0);
    const auto elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    geometry_msgs::msg::Twist command;
    if (result) {
      command.linear.x = result->command.v;
      command.angular.z = result->command.omega;
    }
    command_->publish(command);
    if (!result) {
      return;
    }
    publish_evaluation(*pose, *result, elapsed);
    publish_visualization(*result);
  }

  void publish_evaluation(const Pose & pose, const Solution & result, double elapsed)
  {
    const auto [v, omega] = result.command;
    const std::array values{v, v, 0.0, omega, elapsed};
    for (std::size_t i = 0; i < values.size(); ++i) {
      std_msgs::msg::Float32 scalar;
      scalar.data = static_cast<float>(values[i]);
      telemetry_[i]->publish(scalar);
    }

    mpc_eval_msgs::msg::MPCEval sample;
    sample.header.stamp = now();
    sample.header.frame_id = get_name();
    sample.state_cost = result.cost;
    sample.global_x = pose.x;
    sample.global_y = pose.y;
    sample.global_yaw = pose.yaw;
    sample.cmd_vx = v;
    sample.cmd_yawrate = omega;
    // Keep the existing 4WIDS evaluation fields for the simulated vehicle.
    auto wheel = [v, omega, this](double x, double y, float & steer, float & rotor) {
        const double forward = v - omega * y;
        const double lateral = omega * x;
        steer = std::atan2(lateral, forward);
        rotor = std::hypot(forward, lateral) / tire_radius_;
      };
    wheel(front_, left_, sample.cmd_steer_fl, sample.cmd_rotor_fl);
    wheel(front_, -right_, sample.cmd_steer_fr, sample.cmd_rotor_fr);
    wheel(-rear_, left_, sample.cmd_steer_rl, sample.cmd_rotor_rl);
    wheel(-rear_, -right_, sample.cmd_steer_rr, sample.cmd_rotor_rr);
    sample.calc_time_ms = elapsed;
    sample.goal_reached = result.goal_reached;
    eval_->publish(sample);
  }

  visualization_msgs::msg::Marker line(
    const std::vector<Pose> & trajectory, int id, bool sample)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_;
    marker.header.stamp = now();
    marker.ns = get_name();
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = sample ? 0.015 : 0.05;
    marker.color.r = sample ? 0.4 : 0.0;
    marker.color.g = 0.8;
    marker.color.b = sample ? 0.9 : 0.2;
    marker.color.a = sample ? 0.45 : 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    for (const auto & state : trajectory) {
      geometry_msgs::msg::Point point;
      point.x = state.x;
      point.y = state.y;
      point.z = 0.15;
      marker.points.push_back(point);
    }
    return marker;
  }

  void publish_visualization(const Solution & result)
  {
    if (optimal_->get_subscription_count() > 0) {
      visualization_msgs::msg::MarkerArray markers;
      markers.markers.push_back(line(result.trajectory, 0, false));
      optimal_->publish(markers);
    }
    if (!result.samples.empty()) {
      visualization_msgs::msg::MarkerArray markers;
      for (const auto & sample : result.samples) {
        markers.markers.push_back(line(sample, static_cast<int>(markers.markers.size()), true));
      }
      sampled_->publish(markers);
    }
    if (overlay_->get_subscription_count() > 0) {
      rviz_2d_overlay_msgs::msg::OverlayText label;
      label.width = 500;
      label.height = 50;
      label.vertical_alignment = rviz_2d_overlay_msgs::msg::OverlayText::TOP;
      label.vertical_distance = 50;
      label.text_size = 22.0;
      label.fg_color.g = 1.0;
      label.fg_color.b = 0.9;
      label.fg_color.a = 1.0;
      label.text = get_name();
      overlay_->publish(label);
    }
  }

  const Settings settings_;
  MppiCore core_;
  const double odom_timeout_;
  const double tire_radius_, front_, rear_, left_, right_;
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_;
  std::string frame_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  std::optional<rclcpp::Time> received_;
  grid_map::GridMap collision_, distance_, yaw_;
  bool collision_ready_{}, distance_ready_{}, yaw_ready_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_;
  rclcpp::Publisher<mpc_eval_msgs::msg::MPCEval>::SharedPtr eval_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr optimal_, sampled_;
  rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr overlay_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, 5> telemetry_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr collision_sub_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr distance_sub_, yaw_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace mppi_unicycle

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<mppi_unicycle::SimulationController>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("mppi_unicycle"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
