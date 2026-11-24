#include "mppi_unicycle/action/follow_path.hpp"
#include "ros_inputs.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <misachiya_drive/msg/drive_state.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <string>
#include <utility>
#include <vector>

namespace mppi_unicycle
{
namespace
{
using FollowPath = action::FollowPath;
using Goal = rclcpp_action::ServerGoalHandle<FollowPath>;
using Clock = std::chrono::steady_clock;
using DriveState = misachiya_drive::msg::DriveState;
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
constexpr auto distance_layer = "distance_error";
constexpr std::size_t visualized_samples = 100;

Marker prediction_marker(const rclcpp::Time & stamp, const char * name, int id)
{
  Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = stamp;
  marker.ns = name;
  marker.id = id;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  // The markers disappear when the controller stops solving.
  marker.lifetime.nanosec = 300000000U;
  return marker;
}
}  // namespace

class RouteController : public rclcpp::Node
{
public:
  RouteController()
  : Node("mppi_unicycle"),
    settings_(declare_settings(*this)), core_(settings_),
    base_(declare_parameter<std::string>("base_frame", "base_link")),
    wheel_radius_(positive_parameter(*this, "wheel_radius_m", 0.09)),
    drive_state_timeout_(positive_parameter(*this, "drive_state_timeout", 0.4)),
    startup_timeout_(positive_parameter(*this, "startup_timeout", 2.0)),
    pose_timeout_(positive_parameter(*this, "pose_timeout", 0.4)),
    stall_timeout_(positive_parameter(*this, "stall_timeout", 8.0)),
    compensate_pose_age_(declare_parameter("compensate_pose_age", false)),
    tf_(get_clock()), listener_(tf_, this, false)
  {
    if (base_.empty() || base_.front() == '/') {
      throw std::invalid_argument("Set a valid base_frame");
    }
    RCLCPP_INFO(
      get_logger(), "Wheel speed floor %.3f m/s, limit %.3f m/s", settings_.wheel_speed_floor,
      settings_.wheel_speed_limit);
    if (settings_.wheel_speed_floor == 0.0) {
      RCLCPP_WARN(
        get_logger(), "wheel_speed_floor is 0: wheel commands can fall below the start speed of "
        "sensorless ESCs, which stop with an overspeed fault");
    }
    RCLCPP_INFO(
      get_logger(), "Actuator response: dead time %.3f s, time constant %.3f s, wheel "
      "acceleration limit %.3f m/s^2, deceleration limit %.3f m/s^2, pose age %s",
      settings_.actuator_dead_time, settings_.actuator_time_constant, settings_.wheel_accel_limit,
      settings_.wheel_decel_limit > 0.0 ? settings_.wheel_decel_limit : settings_.wheel_accel_limit,
      compensate_pose_age_ ? "compensated" : "ignored");
    command_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 5);
    predicted_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("mpc/predicted_pose", 5);
    optimal_trajectory_ = create_publisher<MarkerArray>("mpc/optimal_traj", 1);
    sampled_trajectories_ = create_publisher<MarkerArray>("mpc/sampled_traj", 1);
    calculation_time_ = create_publisher<std_msgs::msg::Float32>("mpc/calc_time", 1);
    server_ = rclcpp_action::create_server<FollowPath>(
      this, "follow_path",
      [this](const rclcpp_action::GoalUUID &, const std::shared_ptr<const FollowPath::Goal> goal) {
        const auto & header = goal->path.header;
        if (goal_ || pending_route_ || header.frame_id != "map" || header.stamp.sec < 0 ||
          header.stamp.nanosec >= 1000000000U)
        {
          return rclcpp_action::GoalResponse::REJECT;
        }
        pending_route_ = path_poses(goal->path);
        return pending_route_ ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
          rclcpp_action::GoalResponse::REJECT;
      },
      [this](const std::shared_ptr<Goal> goal) {
        return goal == goal_ ? rclcpp_action::CancelResponse::ACCEPT :
          rclcpp_action::CancelResponse::REJECT;
      },
      [this](const std::shared_ptr<Goal> goal) {
        goal_ = goal;
        route_stamp_ = rclcpp::Time(goal->get_goal()->path.header.stamp).nanoseconds();
        end_ = pending_route_->back();
        core_.set_route(std::move(*pending_route_));
        pending_route_.reset();
        localization_.reset();
        progress_pose_.reset();
        started_ = Clock::now();
      });

    reference_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
      "reference_costmap", rclcpp::QoS(1).transient_local(),
      [this](const grid_map_msgs::msg::GridMap & message) {
        grid_map::GridMap map;
        if (message.header.frame_id == "map" &&
          grid_map::GridMapRosConverter::fromMessage(message, map) &&
          map.exists(distance_layer) && (map.getSize() > 0).all())
        {
          outside_reference_distance_ = map[distance_layer].maxCoeff();
          reference_ = std::move(map);
        } else {
          reference_.reset();
        }
      });

    const auto drive_state_topic = declare_parameter<std::string>("drive_state_topic", "");
    if (!drive_state_topic.empty()) {
      drive_sub_ = create_subscription<DriveState>(
        drive_state_topic, rclcpp::SensorDataQoS().keep_last(1),
        [this](const DriveState & state) {drive_state_ = state;});
    }

    const auto costmap_topic = declare_parameter<std::string>("collision_costmap_topic", "");
    if (!costmap_topic.empty()) {
      costmap_timeout_ = positive_parameter(*this, "costmap_timeout", 0.5);
      collision_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        costmap_topic, 1, [this](const nav_msgs::msg::OccupancyGrid & grid) {
          if (grid_map::GridMapRosConverter::fromOccupancyGrid(grid, "collision_cost", collision_)) {
            collision_received_ = Clock::now();
          } else {
            collision_received_.reset();
          }
        });
    }
    const auto interval = positive_parameter(*this, "control_interval", 0.05);
    timer_ = create_wall_timer(std::chrono::duration<double>(interval), [this] {step();});
  }

private:
  struct Localization
  {
    rclcpp::Time stamp;
    Clock::time_point received;
    Pose pose;
    std::optional<VOmega> velocity;
  };

  bool fresh_localization() const
  {
    if (!localization_) {
      return false;
    }
    const double age = (now() - localization_->stamp).seconds();
    return age >= -0.05 && age <= pose_timeout_ &&
      std::chrono::duration<double>(Clock::now() - localization_->received).count() <= pose_timeout_;
  }

  bool update_localization()
  {
    try {
      const auto transform = tf_.lookupTransform("map", base_, tf2::TimePointZero);
      const auto pose = planar_pose(transform.transform);
      const auto & stamp = transform.header.stamp;
      if (!pose || stamp.sec < 0 || stamp.nanosec >= 1000000000U ||
        (stamp.sec == 0 && stamp.nanosec == 0))
      {
        return false;
      }
      const rclcpp::Time measured(stamp, get_clock()->get_clock_type());
      if (localization_ && measured == localization_->stamp) {
        return fresh_localization();
      }

      std::optional<VOmega> velocity;
      if (localization_) {
        const double dt = (measured - localization_->stamp).seconds();
        if (dt > 0.0 && dt <= pose_timeout_) {
          const auto & previous = localization_->pose;
          const double turn = wrap(pose->yaw - previous.yaw);
          const double heading = previous.yaw + 0.5 * turn;
          const double v =
            ((pose->x - previous.x) * std::cos(heading) +
            (pose->y - previous.y) * std::sin(heading)) / dt;
          const VOmega instant{
            std::clamp(v, -settings_.max_linear_speed, settings_.max_linear_speed),
            std::clamp(turn / dt, -settings_.max_angular_speed, settings_.max_angular_speed)};
          const auto last = localization_->velocity.value_or(instant);
          const double weight = dt / (0.12 + dt);
          velocity = VOmega{last.v + weight * (instant.v - last.v),
            last.omega + weight * (instant.omega - last.omega)};
        }
      }
      localization_ = Localization{measured, Clock::now(), *pose, velocity};
      return fresh_localization();
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  bool fresh_costmap() const
  {
    return !collision_sub_ ||
      (collision_received_ && std::chrono::duration<double>(
        Clock::now() - *collision_received_).count() <= costmap_timeout_);
  }

  bool fresh_drive_state() const
  {
    if (!drive_sub_) {
      return true;
    }
    if (!drive_state_ || !drive_state_->left.negotiated || !drive_state_->right.negotiated) {
      return false;
    }
    const auto & stamp = drive_state_->stamp;
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000U || (stamp.sec == 0 && stamp.nanosec == 0)) {
      return false;
    }
    const double age = (now() - rclcpp::Time(stamp, get_clock()->get_clock_type())).seconds();
    return age >= -0.05 && age <= drive_state_timeout_;
  }

  bool apply_wheel_limits()
  {
    if (!drive_sub_) {
      return true;
    }
    const auto & left = drive_state_->left;
    const auto & right = drive_state_->right;
    const double minimum = std::max(
      settings_.wheel_speed_floor,
      wheel_radius_ * std::max(left.min_output_rad_s, right.min_output_rad_s));
    double maximum = wheel_radius_ * std::min(left.max_output_rad_s, right.max_output_rad_s);
    if (settings_.wheel_speed_limit > 0.0) {
      maximum = std::min(maximum, settings_.wheel_speed_limit);
    }
    return core_.set_wheel_speed_limits(minimum, maximum);
  }

  std::optional<Pose> costmap_transform()
  {
    if (!collision_sub_ || collision_.getFrameId() == "map") {
      return Pose{};
    }
    try {
      return planar_pose(tf_.lookupTransform(collision_.getFrameId(), "map",
          tf2::TimePointZero).transform);
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  void publish(VOmega input = {})
  {
    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = now();
    command.header.frame_id = base_;
    command.twist.linear.x = input.v;
    command.twist.angular.z = input.omega;
    command_->publish(command);
    // The solver replays what the chassis received, including the stops sent outside a solve.
    core_.command_sent(input, rclcpp::Time(command.header.stamp).seconds());
  }

  void publish_predictions(const rclcpp::Time & start_time)
  {
    const auto & optimal = core_.optimal_trajectory();
    if (optimal.empty()) {
      return;
    }
    // Where the solver expects the robot when its command starts to act. Localization at that
    // time shows how far the prediction is off.
    geometry_msgs::msg::PoseStamped start;
    start.header.frame_id = "map";
    start.header.stamp = start_time;
    start.pose.position.x = optimal.front().x;
    start.pose.position.y = optimal.front().y;
    start.pose.orientation.z = std::sin(0.5 * optimal.front().yaw);
    start.pose.orientation.w = std::cos(0.5 * optimal.front().yaw);
    predicted_pose_->publish(start);

    std_msgs::msg::Float32 time;
    time.data = static_cast<float>(core_.calculation_time_ms());
    calculation_time_->publish(time);

    const rclcpp::Time stamp = now();
    if (optimal_trajectory_->get_subscription_count() > 0) {
      MarkerArray arrows;
      arrows.markers.reserve(optimal.size());
      for (const auto & pose : optimal) {
        auto arrow = prediction_marker(
          stamp, "optimal_trajectory", static_cast<int>(arrows.markers.size()));
        arrow.type = Marker::ARROW;
        arrow.pose.position.x = pose.x;
        arrow.pose.position.y = pose.y;
        arrow.pose.position.z = 0.02;
        arrow.pose.orientation.z = std::sin(0.5 * pose.yaw);
        arrow.pose.orientation.w = std::cos(0.5 * pose.yaw);
        arrow.scale.x = 0.04;
        arrow.scale.y = 0.012;
        arrow.scale.z = 0.012;
        arrow.color.r = 1.0f;
        arrow.color.a = 1.0f;
        arrows.markers.push_back(arrow);
      }
      optimal_trajectory_->publish(arrows);
    }
    if (sampled_trajectories_->get_subscription_count() > 0) {
      MarkerArray lines;
      for (const auto & trajectory : core_.elite_trajectories(visualized_samples)) {
        auto line = prediction_marker(
          stamp, "sampled_trajectories", static_cast<int>(lines.markers.size()));
        line.type = Marker::LINE_STRIP;
        line.scale.x = 0.004;
        line.color.g = 0.35f;
        line.color.b = 1.0f;
        line.color.a = 0.5f;
        line.points.reserve(trajectory.size());
        for (const auto & pose : trajectory) {
          geometry_msgs::msg::Point point;
          point.x = pose.x;
          point.y = pose.y;
          point.z = 0.01;
          line.points.push_back(point);
        }
        lines.markers.push_back(line);
      }
      sampled_trajectories_->publish(lines);
    }
  }

  void finish(const std::string & message, bool reached = false)
  {
    publish();
    auto result = std::make_shared<FollowPath::Result>();
    result->message = message;
    if (goal_->is_canceling()) {
      goal_->canceled(result);
    } else if (reached) {
      goal_->succeed(result);
    } else {
      goal_->abort(result);
    }
    goal_.reset();
  }

  void wait_for_input(const std::string & message)
  {
    if (progress_pose_ ||
      std::chrono::duration<double>(Clock::now() - started_).count() > startup_timeout_)
    {
      finish(message);
    } else {
      publish();
    }
  }

  void step()
  {
    if (!goal_) {
      return;
    }
    if (goal_->is_canceling()) {
      finish("Canceled the recorded route");
      return;
    }
    if (!reference_ || reference_->getTimestamp() != route_stamp_) {
      wait_for_input("The reference costmap for the recorded path is unavailable");
      return;
    }
    if (!fresh_drive_state()) {
      wait_for_input("Chassis state is missing, stale or waiting for the ESC speed limits");
      return;
    }
    if (!apply_wheel_limits()) {
      finish("The negotiated ESC speed limits conflict with the MPPI speed limits");
      return;
    }
    if (!update_localization() || !localization_->velocity) {
      wait_for_input("Localization TF is missing or stale");
      return;
    }
    if (!fresh_costmap()) {
      wait_for_input("The collision costmap is missing or stale");
      return;
    }
    const auto to_costmap = costmap_transform();
    if (!to_costmap) {
      wait_for_input("TF to the collision costmap frame is unavailable");
      return;
    }

    const auto & pose = localization_->pose;
    const auto & measured = *localization_->velocity;
    const double c = std::cos(to_costmap->yaw), s = std::sin(to_costmap->yaw);
    // The solver calls this from its rollout threads. It only reads the maps.
    const auto evaluate_cost = [&](const Pose & state) {
        MapCost cost;
        const grid_map::Position position(state.x, state.y);
        cost.distance = reference_->isInside(position) ?
          reference_->atPosition(
          distance_layer, position, grid_map::InterpolationMethods::INTER_LINEAR) :
          outside_reference_distance_;
        if (collision_sub_) {
          cost.collision = collision_at(collision_, {
            to_costmap->x + c * state.x - s * state.y,
            to_costmap->y + s * state.x + c * state.y});
        }
        return cost;
      };

    // The command leaves after the solve, which takes about as long as the last one did.
    const rclcpp::Time command_time =
      now() + rclcpp::Duration::from_seconds(1e-3 * core_.calculation_time_ms());
    const double sent = command_time.seconds();
    const double observed = compensate_pose_age_ ?
      std::clamp(localization_->stamp.seconds(), sent - pose_timeout_, sent) : sent;
    const auto result = core_.solve(pose, measured, evaluate_cost, Timing{observed, sent});
    if (!result) {
      finish("MPPI did not find a feasible trajectory");
      return;
    }
    publish_predictions(
      command_time + rclcpp::Duration::from_seconds(settings_.actuator_dead_time));
    if (!fresh_localization() || !fresh_costmap() || !fresh_drive_state()) {
      finish("A navigation input expired during optimization");
      return;
    }
    if (result->goal_reached && std::abs(measured.v) < 0.04 && std::abs(measured.omega) < 0.15) {
      finish("Reached the recorded route goal", true);
      return;
    }

    const auto time = Clock::now();
    if (!progress_pose_ || distance(pose, *progress_pose_) >= 0.05 ||
      std::abs(wrap(pose.yaw - progress_pose_->yaw)) >= 0.15)
    {
      progress_pose_ = pose;
      last_progress_ = time;
    } else if (std::chrono::duration<double>(time - last_progress_).count() > stall_timeout_) {
      const auto reason = drive_state_ ? " (chassis: " + drive_state_->reason + ")" : "";
      finish("The chassis stopped making progress along the recorded route" + reason);
      return;
    }

    publish(result->command);
    auto feedback = std::make_shared<FollowPath::Feedback>();
    feedback->distance_to_goal = static_cast<float>(distance(pose, end_));
    goal_->publish_feedback(feedback);
  }

  const Settings settings_;
  MppiCore core_;
  const std::string base_;
  const double wheel_radius_, drive_state_timeout_, startup_timeout_, pose_timeout_, stall_timeout_;
  const bool compensate_pose_age_;
  double costmap_timeout_{};
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_;
  std::shared_ptr<Goal> goal_;
  std::optional<std::vector<Pose>> pending_route_;
  grid_map::Time route_stamp_{};
  Pose end_;
  std::optional<Localization> localization_;
  std::optional<Pose> progress_pose_;
  Clock::time_point started_, last_progress_;
  std::optional<grid_map::GridMap> reference_;
  double outside_reference_distance_{};
  grid_map::GridMap collision_;
  std::optional<Clock::time_point> collision_received_;
  std::optional<DriveState> drive_state_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr predicted_pose_;
  rclcpp::Publisher<MarkerArray>::SharedPtr optimal_trajectory_, sampled_trajectories_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr calculation_time_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr reference_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr collision_sub_;
  rclcpp::Subscription<DriveState>::SharedPtr drive_sub_;
  rclcpp_action::Server<FollowPath>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace mppi_unicycle

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<mppi_unicycle::RouteController>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("mppi_unicycle"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
