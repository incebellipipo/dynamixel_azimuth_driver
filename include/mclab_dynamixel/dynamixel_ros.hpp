#pragma once  // Favor using this over the #ifndef, #define method

#include "chrono"
#include "functional"
#include "memory"
#include "string"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

#include "mclab_dynamixel/dynamixel_ctrl.hpp"

struct PortConfig
{
    std::string port;
    int baudrate;
};

struct ServoConfig {
    std::string name;
    std::string frame_id;
    std::string control_topic;
    int device_id;
    float gain;
    float delta;
    float offset;
};

class DynamixelServoRos;

class DynamixelRos : public rclcpp::Node
{
private:
    void f_param_digest();

    void f_initialize_servos();

    PortConfig port_config_;

    std::vector<ServoConfig> servo_configs_;

    std::vector<std::shared_ptr<DynamixelServoRos>> servos_;

protected:

    std::shared_ptr<DynamixelCtrl> dynamixel_ctrl_;


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

    void initialize();

};

class DynamixelServoRos
{
private:

    rclcpp::Node::SharedPtr node_;

    float rate_gain_;

    float saturation_gain_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;

    float desired_angle_;

    std::shared_ptr<ServoCtrl> servo_;

    ServoConfig servo_config_;

    void timer_callback();

    void angle_callback(const std_msgs::msg::Float32::SharedPtr msg);

    void update_parameters();
public:

    DynamixelServoRos(rclcpp::Node::SharedPtr node, ServoConfig servo_config, std::shared_ptr<DynamixelCtrl> ctrl_interface);

};