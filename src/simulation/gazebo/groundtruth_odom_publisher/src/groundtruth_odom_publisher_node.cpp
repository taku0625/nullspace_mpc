#include "groundtruth_odom_publisher/groundtruth_odom_publisher.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gazebo::GroundTruthOdomPublisher>());
    rclcpp::shutdown();
    return 0;
}
