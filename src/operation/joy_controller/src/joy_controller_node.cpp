#include "joy_controller/joy_controller.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<operation::JoyController>());
    rclcpp::shutdown();
    return 0;
}
