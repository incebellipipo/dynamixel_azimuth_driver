#include "dynamixel_servo/dynamixel_ros.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DynamixelRos>();

    node->initialize();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}