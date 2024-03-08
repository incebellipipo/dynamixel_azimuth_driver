#pragma once  // Favor using this over the #ifndef, #define method

#include "chrono"
#include "functional"
#include "memory"
#include "string"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

#include "mclab_dynamixel/dynamixel_ctrl.hpp"

struct PortParameter
{
    std::string port;
    int baudrate;
};

class DynamixelServoRos;

class DynamixelRos : public rclcpp::Node
{


protected:

    std::shared_ptr<DynamixelCtrl> dynamixel_ctrl_;

    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback();

    PortParameter port_param_;

    void declare_parameters();

    void update_parameters();


    rclcpp::executors::StaticSingleThreadedExecutor executor_;

public:
    /**
     * @brief Construct a new Minimal Publisher object
     *
     * The constructor initializes the publisher and timer.
     */
    DynamixelRos();

    // return the executor
    auto get_executor() -> decltype(executor_)& {
        return executor_;
    }

    std::vector<std::shared_ptr<DynamixelServoRos>> servo_ros;

    void run();
};

class DynamixelServoRos : public rclcpp::Node
{
private:

    float rate_gain_;

    float saturation_gain_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;

    float desired_angle_;

    std::shared_ptr<ServoCtrl> servo_;

    void timer_callback();

    void angle_callback(const std_msgs::msg::Float32::SharedPtr msg);

    void declare_parameters();

    void update_parameters();
public:
    DynamixelServoRos(std::shared_ptr<ServoCtrl> servo, std::string name);
};