#include "mppi_unicycle/action/follow_path.hpp"
#include "ros_inputs.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <algorithm>
#include <chrono>
#include <string>
#include <utility>

namespace mppi_unicycle
{
namespace
{
using FollowPath = action::FollowPath;
using Goal = rclcpp_action::ServerGoalHandle<FollowPath>;
using Clock = std::chrono::steady_clock;
}  // namespace

class RouteController : public rclcpp::Node
{
public:
  RouteController()
  : Node("mppi_unicycle"),
    settings_(declare_settings(*this)), core_(settings_),
    base_(declare_parameter<std::string>("base_frame", "base_link")),
    pose_timeout_(positive_parameter(*this, "pose_timeout", 0.4)),
    stall_timeout_(positive_parameter(*this, "stall_timeout", 8.0)),
    tf_(get_clock()), listener_(tf_, this, false)
  {
    if (base_.empty() || base_.front() == '/') {
      throw std::invalid_argument("Set a valid base_frame");
    }
    command_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 5);
    server_ = rclcpp_action::create_server<FollowPath>(
      this, "follow_path",
      [this](const rclcpp_action::GoalUUID &, const std::shared_ptr<const FollowPath::Goal> goal) {
        if (goal_ || pending_route_ || goal->path.header.frame_id == base_) {
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
        frame_ = goal->get_goal()->path.header.frame_id;
        end_ = pending_route_->back();
        core_.set_route(std::move(*pending_route_));
        pending_route_.reset();
        localization_.reset();
        progress_pose_.reset();
        started_ = Clock::now();
      });

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
      const auto transform = tf_.lookupTransform(frame_, base_, tf2::TimePointZero);
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

  std::optional<Pose> costmap_transform()
  {
    if (!collision_sub_ || collision_.getFrameId() == frame_) {
      return Pose{};
    }
    try {
      return planar_pose(tf_.lookupTransform(collision_.getFrameId(), frame_,
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
      std::chrono::duration<double>(Clock::now() - started_).count() > pose_timeout_)
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
    Costmap evaluate_cost;
    if (collision_sub_) {
      const double c = std::cos(to_costmap->yaw), s = std::sin(to_costmap->yaw);
      evaluate_cost = [this, to_costmap, c, s](const Pose & state) {
          const grid_map::Position position(
            to_costmap->x + c * state.x - s * state.y,
            to_costmap->y + s * state.x + c * state.y);
          return MapCost{collision_at(collision_, position), {}, {}};
        };
    }

    const auto result = core_.solve(pose, measured, evaluate_cost);
    if (!result) {
      finish("MPPI did not find a feasible trajectory");
      return;
    }
    if (!fresh_localization() || !fresh_costmap()) {
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
      finish("The chassis stopped making progress along the recorded route");
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
  const double pose_timeout_, stall_timeout_;
  double costmap_timeout_{};
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_;
  std::shared_ptr<Goal> goal_;
  std::optional<std::vector<Pose>> pending_route_;
  std::string frame_;
  Pose end_;
  std::optional<Localization> localization_;
  std::optional<Pose> progress_pose_;
  Clock::time_point started_, last_progress_;
  grid_map::GridMap collision_;
  std::optional<Clock::time_point> collision_received_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr collision_sub_;
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
