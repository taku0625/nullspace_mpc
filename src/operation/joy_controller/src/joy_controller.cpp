#include "joy_controller/joy_controller.hpp"

#include <cmath>

namespace operation
{

//constructor
JoyController::JoyController()
    : Node("joy_controller")
{
    // load parameters 

    //// joy stick axes
    joy_top_left_button_idx = declare_parameter<int>("joy_top_left_button_idx", 4);
    joy_top_right_button_idx = declare_parameter<int>("joy_top_right_button_idx", 5);
    joy_left_stick_x_idx = declare_parameter<int>("joy_left_stick_x_idx", 0);
    joy_left_stick_y_idx = declare_parameter<int>("joy_left_stick_y_idx", 1);
    joy_right_stick_x_idx = declare_parameter<int>("joy_right_stick_x_idx", 3);
    joy_right_stick_y_idx = declare_parameter<int>("joy_right_stick_y_idx", 4);

    //// max command values
    abs_max_linear_vel_x = declare_parameter<double>("abs_max_linear_vel_x", 3.0);
    abs_max_linear_vel_y = declare_parameter<double>("abs_max_linear_vel_y", 3.0);
    abs_max_angular_vel_z = declare_parameter<double>("abs_max_angular_vel_z", 1.0);

    //// subscribing topic names
    const auto joy_topic = declare_parameter<std::string>("joy_topic", "/joy");

    //// publishing topic names
    const auto control_cmd_vel_topic = declare_parameter<std::string>("control_cmd_vel_topic", "/cmd_vel");

    // initialize subscribers
    sub_joy_ = create_subscription<sensor_msgs::msg::Joy>(
        joy_topic, rclcpp::SensorDataQoS().keep_last(1),
        [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) { joyCallback(msg); });

    // initialize publishers
    pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>(control_cmd_vel_topic, 10);
}

// /joy topic callback
void JoyController::joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
{
    const auto valid_axis = [&msg](int index) {
        return index >= 0 && static_cast<size_t>(index) < msg->axes.size()
            && std::isfinite(msg->axes[index]);
    };
    const auto valid_button = [&msg](int index) {
        return index >= 0 && static_cast<size_t>(index) < msg->buttons.size();
    };
    if (!valid_axis(joy_left_stick_x_idx) || !valid_axis(joy_left_stick_y_idx)
        || !valid_axis(joy_right_stick_x_idx) || !valid_axis(joy_right_stick_y_idx)
        || !valid_button(joy_top_left_button_idx) || !valid_button(joy_top_right_button_idx))
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Joystick data does not match the configured inputs.");
        pub_cmd_vel_->publish(geometry_msgs::msg::Twist{});
        return;
    }

    // parse received joy message
    float scale = 1.0f;
    if (msg->buttons[joy_top_left_button_idx]) // scale down the command (0.5x)
    {
        scale = scale * 0.5f;
    }
    if (msg->buttons[joy_top_right_button_idx]) // scale up the command (2.0x)
    {
        scale = scale * 2.0f;
    }
    float val_left_stick_x  = scale * msg->axes[joy_left_stick_x_idx] * (-1.0f);
    float val_left_stick_y  = scale * msg->axes[joy_left_stick_y_idx];
    float val_right_stick_x = scale * msg->axes[joy_right_stick_x_idx];
    float val_right_stick_y = scale * msg->axes[joy_right_stick_y_idx];

    // [for debug] annouce received joy message
    RCLCPP_DEBUG(get_logger(), "Received Joy Command: Lstick_x = %+5.1f, Lstick_y = %+5.1f, Rstick_x = %+5.1f, Rstick_y = %+5.1f", val_left_stick_x, val_left_stick_y, val_right_stick_x, val_right_stick_y);

    // calculate Twist message to publish
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x =  abs_max_linear_vel_x * val_left_stick_y; // forward : positive, backward : negative
    cmd_vel.linear.y = -abs_max_linear_vel_y * val_left_stick_x; // left    : positive, right    : negative
    cmd_vel.linear.z =  0.0f;
    cmd_vel.angular.x = 0.0f;
    cmd_vel.angular.y = 0.0f;
    cmd_vel.angular.z = abs_max_angular_vel_z * val_right_stick_x; // left    : positive, right    : negative

    // [for debug] annouce publishing Twist message
    RCLCPP_DEBUG(get_logger(), "Send Twist Command: linear_x = %+5.1f, linear_y = %+5.1f, angular_z = %+5.1f", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // publish Twist message
    pub_cmd_vel_->publish(cmd_vel);
}

} // namespace operation
