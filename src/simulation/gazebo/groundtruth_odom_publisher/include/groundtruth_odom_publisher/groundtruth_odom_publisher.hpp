#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace gazebo
{
    class GroundTruthOdomPublisher : public rclcpp::Node
    {
        public:
            GroundTruthOdomPublisher();
            void groundTruthOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
        private:
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_groundtruth_odom_;
            tf2_ros::TransformBroadcaster tf_broadcaster_;
            geometry_msgs::msg::TransformStamped odom_tf_;
            std::string base_frame_name, odom_frame_name;
    };
}
