#include "groundtruth_odom_publisher/groundtruth_odom_publisher.hpp"

namespace gazebo
{

// constructor
GroundTruthOdomPublisher::GroundTruthOdomPublisher()
    : Node("groundtruth_odom_publisher"), tf_broadcaster_(*this)
{
    // load parameters 

    //// frame names
    base_frame_name = declare_parameter<std::string>("base_frame", "base_link");
    odom_frame_name = declare_parameter<std::string>("odom_frame", "odom");

    //// subscribing topic names
    const auto groundtruth_odom_topic = declare_parameter<std::string>("groundtruth_odom_topic", "/groundtruth_odom");

    // initialize subscribers
    sub_groundtruth_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        groundtruth_odom_topic, rclcpp::SensorDataQoS().keep_last(1),
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) { groundTruthOdomCallback(msg); });
}

// /odom topic callback
void GroundTruthOdomPublisher::groundTruthOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
    odom_tf_.header.stamp = msg->header.stamp;
    odom_tf_.header.frame_id = odom_frame_name;
    odom_tf_.child_frame_id = base_frame_name;
    odom_tf_.transform.translation.x = msg->pose.pose.position.x;
    odom_tf_.transform.translation.y = msg->pose.pose.position.y;
    odom_tf_.transform.translation.z = msg->pose.pose.position.z;
    odom_tf_.transform.rotation.x = msg->pose.pose.orientation.x;
    odom_tf_.transform.rotation.y = msg->pose.pose.orientation.y;
    odom_tf_.transform.rotation.z = msg->pose.pose.orientation.z;
    odom_tf_.transform.rotation.w = msg->pose.pose.orientation.w;
    tf_broadcaster_.sendTransform(odom_tf_);
}

} // namespace gazebo
