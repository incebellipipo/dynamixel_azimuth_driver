#include "mclab_dynamixel/dynamixel_ros.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::StaticSingleThreadedExecutor executor;

    auto node = std::make_shared<DynamixelRos>();
    executor.add_node(node);
    for(auto servo : node->servo_ros) {
        node->get_executor().add_node(servo);
    }
    node->get_executor().spin();
    rclcpp::shutdown();
    return 0;
}