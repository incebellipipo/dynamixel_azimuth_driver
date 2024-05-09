#include "mclab_dynamixel/dynamixel_ros.hpp"
#include "rclcpp/executor.hpp"

#include "cmath"
using namespace std::chrono_literals;


DynamixelRos::DynamixelRos() : Node("dynamixel_ros")
{
    this->declare_parameters();

    this->f_param_digest();

    dynamixel_ctrl_ = std::make_shared<DynamixelCtrl>();
    dynamixel_ctrl_->init(
        port_param_.port,
        port_param_.baudrate
    );

    dynamixel_ctrl_->scanDevices();

    // for(auto id : dynamixel_ctrl_->getDeviceList()) {
    //     std::cout << "Found servo with id: " << (int)id << std::endl;
    //     auto servo = std::make_shared<DynamixelServoRos>(
    //         std::make_shared<ServoCtrl>(id, dynamixel_ctrl_), "servo_" + std::to_string(id));
    //     servo_ros.push_back(servo);
    // }

    this->f_initialize_servos();

    timer_ = this->create_wall_timer(
        20ms, std::bind(&DynamixelRos::timer_callback, this));



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
    this->declare_parameter("hardware.port", "/dev/ttyUSB0");
    this->declare_parameter("hardware.baudrate", 57600);
}

void DynamixelRos::f_param_digest() {
    std::cout << "reading the parameters" << std::endl;

    this->get_parameter("hardware.port", port_param_.port);
    this->get_parameter("hardware.baudrate", port_param_.baudrate);

    std::map<std::string, rclcpp::Parameter> parameter_map;

    this->get_parameters("servos", parameter_map);
    std::set<std::string> servo_names;
    for(auto & key_value : parameter_map){
        size_t pos = key_value.first.find('.');
        auto servo_name = key_value.first.substr(0, pos);
        servo_names.insert(servo_name);
    }

    for(auto & servo_name : servo_names){

        ServoConfig servo_config;

        this->get_parameter_or<std::string>("servos." + servo_name + ".frame_id", servo_config.frame_id, std::string("servo_" + servo_name + "_link"));
        this->get_parameter_or<std::string>("servos." + servo_name + ".control_topic", servo_config.control_topic, std::string("servo_" + servo_name + "/control"));
        this->get_parameter_or<int>("servos." + servo_name + ".device_id", servo_config.device_id, -1);
        this->get_parameter_or<float>("servos." + servo_name + ".gain", servo_config.gain, 1.0);
        this->get_parameter_or<float>("servos." + servo_name + ".delta", servo_config.delta, 0.1);
        this->get_parameter_or<float>("servos." + servo_name + ".offset", servo_config.offset, 0.0);

        servo_configs_.push_back(servo_config);
    }
}

void DynamixelRos::f_initialize_servos()
{

    auto device_ids = dynamixel_ctrl_->getDeviceList();

    for(auto id : device_ids) {
        RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Found servo with id: " << id);
    }

    for(auto & servo_config : servo_configs_) {
        if(servo_config.device_id == -1) {
            std::cerr << "No device id specified for servo " << servo_config.name << std::endl;
            continue;
        }

        if(std::find(device_ids.begin(), device_ids.end(), servo_config.device_id) == device_ids.end()) {

            RCLCPP_ERROR_STREAM_ONCE(this->get_logger(),
                "Device id " << servo_config.device_id << " not found"
            );

            continue;
        }

        auto servo = std::make_shared<DynamixelServoRos>(servo_config, dynamixel_ctrl_);
        servo_ros.push_back(servo);
    }

}

DynamixelServoRos::DynamixelServoRos(ServoConfig config, std::shared_ptr<DynamixelCtrl> ctrl_interface)
    : Node("dynamixel_" + config.name), servo_config_(config)
{
    this->declare_parameters();
    this->update_parameters();

    servo_ = std::make_shared<ServoCtrl>(servo_config_.device_id, ctrl_interface);

    timer_ = this->create_wall_timer(
        20ms, std::bind(&DynamixelServoRos::timer_callback, this));

    subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        servo_config_.control_topic, 10, std::bind(&DynamixelServoRos::angle_callback, this, std::placeholders::_1));

    servo_->ping();
    servo_->init();

}

void DynamixelServoRos::timer_callback()
{
    servo_->update();
    update_parameters();

    float diff = desired_angle_ - servo_->getPresentState().position + servo_config_.offset;

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