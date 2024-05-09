#include "mclab_dynamixel/dynamixel_ros.hpp"
#include "rclcpp/executor.hpp"

#include "cmath"
using namespace std::chrono_literals;


DynamixelRos::DynamixelRos() :
    Node("dynamixel_ros",
        rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true))
{
    this->f_param_digest();
}

void DynamixelRos::initialize()
{
    dynamixel_ctrl_ = std::make_shared<DynamixelCtrl>();
    dynamixel_ctrl_->init(
        port_config_.port,
        port_config_.baudrate
    );

    dynamixel_ctrl_->scanDevices();


    this->f_initialize_servos();


}



void DynamixelRos::f_param_digest() {
    std::cout << "reading the parameters" << std::endl;

    this->get_parameter("hardware.port", port_config_.port);
    this->get_parameter("hardware.baudrate", port_config_.baudrate);

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

        auto servo = std::make_shared<DynamixelServoRos>(shared_from_this() ,servo_config, dynamixel_ctrl_);

        servos_.push_back(servo);
    }

}

DynamixelServoRos::DynamixelServoRos(std::shared_ptr<rclcpp::Node> node, ServoConfig config, std::shared_ptr<DynamixelCtrl> ctrl_interface)
    : node_(node), servo_config_(config)
{
    this->update_parameters();

    servo_ = std::make_shared<ServoCtrl>(servo_config_.device_id, ctrl_interface);

    timer_ = node_->create_wall_timer(
        20ms, std::bind(&DynamixelServoRos::timer_callback, this));

    subscriber_ = node_->create_subscription<std_msgs::msg::Float32>(
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


void DynamixelServoRos::update_parameters()
{

    node_->get_parameter_or<float>("servos." + servo_config_.name + ".gain", servo_config_.gain, 1.0);
    node_->get_parameter_or<float>("servos." + servo_config_.name + ".delta", servo_config_.delta, 0.1);
    node_->get_parameter_or<float>("servos." + servo_config_.name + ".offset", servo_config_.offset, 0.0);

}