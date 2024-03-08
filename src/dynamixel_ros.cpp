#include "mclab_dynamixel/dynamixel_ros.hpp"
#include "rclcpp/executor.hpp"

#include "cmath"
using namespace std::chrono_literals;


DynamixelRos::DynamixelRos()
    : Node("dynamixel_ros")
{
    declare_parameters();
    update_parameters();
    timer_ = this->create_wall_timer(
        20ms, std::bind(&DynamixelRos::timer_callback, this));


    dynamixel_ctrl_ = std::make_shared<DynamixelCtrl>();
    dynamixel_ctrl_->init(
        port_param_.port,
        port_param_.baudrate
    );

    dynamixel_ctrl_->scanDevices();

    for(auto id : dynamixel_ctrl_->getDeviceList()) {
        std::cout << "Found servo with id: " << (int)id << std::endl;
        auto servo = std::make_shared<DynamixelServoRos>(
            std::make_shared<ServoCtrl>(id, dynamixel_ctrl_), "servo_" + std::to_string(id));
        servo_ros.push_back(servo);
    }

}

// implement the timer
void DynamixelRos::timer_callback()
{
}

void DynamixelRos::run()
{
    executor_.spin();
}



void DynamixelRos::declare_parameters()
{
    this->declare_parameter("port", "/dev/ttyUSB0");
    this->declare_parameter("baudrate", 57600);
}

void DynamixelRos::update_parameters()
{
    this->get_parameter("port", port_param_.port);
    this->get_parameter("baudrate", port_param_.baudrate);
}

DynamixelServoRos::DynamixelServoRos(std::shared_ptr<ServoCtrl> servo, std::string name)
    : Node(name)
{
    this->declare_parameters();
    this->update_parameters();

    servo_ = servo;

    timer_ = this->create_wall_timer(
        20ms, std::bind(&DynamixelServoRos::timer_callback, this));

    subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        "servo_" + std::to_string(servo_->getId()) + "/desired_angle", 10, std::bind(&DynamixelServoRos::angle_callback, this, std::placeholders::_1));

    servo_->ping();
    servo_->init();

}

void DynamixelServoRos::timer_callback()
{
    servo_->update();
    update_parameters();

    float diff = desired_angle_ - servo_->getPresentState().position;

    float v = atan2(sin(diff), cos(diff));
    auto gain = rate_gain_ * v / (fabs(v) + saturation_gain_);

    servo_->setGoalVelocity(gain);

}

void DynamixelServoRos::angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    desired_angle_ = msg->data;
}
void DynamixelServoRos::declare_parameters()
{
    this->declare_parameter("rate", 10.0);
    this->declare_parameter("saturation", 10.0);
}

void DynamixelServoRos::update_parameters()
{
    rate_gain_ = this->get_parameter("rate").as_double();
    saturation_gain_ = this->get_parameter("saturation").as_double();
}