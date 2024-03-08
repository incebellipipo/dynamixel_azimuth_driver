#include "mclab_dynamixel/dynamixel_ctrl.hpp"

#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

#include "iostream"
#include "vector"
#include "cmath"


void DynamixelCtrl::init(std::string port, int baudrate) {
    this->port_ = port;
    this->baudrate_ = baudrate;

    dxl_wb_ = std::make_shared<DynamixelWorkbench>();

    const char* log;
    auto result = dxl_wb_->init(port.c_str(), baudrate, &log);
    if (result == false) {
        std::cerr << log << std::endl;
    }
    dxl_wb_->begin();
}

void DynamixelCtrl::scanDevices() {

    uint8_t get_id[16];
    uint8_t get_the_number_of_id;

    const char *log;
    auto result = this->dxl_wb_->scan(get_id, &get_the_number_of_id, 253, &log);
    if(result == false) {
        std::cerr << log << std::endl;
    }

    dxl_id_list_.assign(get_id, get_id + get_the_number_of_id);
}

auto DynamixelCtrl::getDeviceList() -> decltype(dxl_id_list_) {
    return dxl_id_list_;
}

auto DynamixelCtrl::getCtrlInterface() -> decltype(dxl_wb_) {
    return dxl_wb_;
}

ServoCtrl::ServoCtrl(
        int id,
        std::shared_ptr<DynamixelCtrl> ctrl_interface
    ): ctrl_interface_(ctrl_interface) {

    id_ = id;

}

auto ServoCtrl::getPresentState() -> decltype(present_state_) {
    return present_state_;
}

void ServoCtrl::init() {

    setWheelMode();
    setTorque(true);

}

auto ServoCtrl::getId() -> decltype(id_) {
    return id_;
}

/**
 * @brief Update the servo state
 *
*/
void ServoCtrl::update() {
    getPresentPosition();
    getPresentVelocity();
    getPresentLoad();
    getPresentVoltage();
    getPresentTemperature();
}



void ServoCtrl::setGoalPosition(float goal_position) {
    int32_t goal = ctrl_interface_->getCtrlInterface()->convertRadian2Value(id_, goal_position);
    ctrl_interface_->getCtrlInterface()->goalPosition(id_, goal);
}

/**
 * @brief Set the Goal Velocity object
 *
 * @param goal_velocity : rad/s
*/
void ServoCtrl::setGoalVelocity(float goal_velocity) {
    const char *log;
    int32_t value = ctrl_interface_->getCtrlInterface()->convertVelocity2Value(id_, goal_velocity);
    if (goal_velocity > 0) {
        value = value - 1023;
    }
    auto result = ctrl_interface_->getCtrlInterface()->goalVelocity(id_, value, &log);
    if (result == false) {
        std::cerr << log << std::endl;
    }
}

/**
 * @brief Set the Torque object
 *
 * @param torque : true for on, false for off
*/
void ServoCtrl::setTorque(bool torque) {
    const char *log;
    if (torque) {
        ctrl_interface_->getCtrlInterface()->torqueOn(id_, &log);
    } else {
        ctrl_interface_->getCtrlInterface()->torqueOff(id_, &log);
    }
}

/**
 * @brief Set the Wheel Mode object
 *
*/
void ServoCtrl::setWheelMode() {
    const char *log;
    auto result = ctrl_interface_->getCtrlInterface()->wheelMode(id_, 0, &log);
    if (result == false) {
        std::cerr << log << std::endl;
    }
}

/**
 * @brief Set the Joint Mode object
 *
*/
void ServoCtrl::setJointMode() {
    const char *log;
    auto result = ctrl_interface_->getCtrlInterface()->jointMode(id_, 0, 0, &log);
    if (result == false) {
        std::cerr << log << std::endl;
    }
}

/**
 * @brief Get the Present Position object
 *
*/
void ServoCtrl::getPresentPosition() {
    const char *log;
    auto result = ctrl_interface_->getCtrlInterface()->getRadian(id_, &present_state_.position, &log);
    if (result == false) {
        std::cerr << log << std::endl;
    }
}

/**
 * @brief Get the Present Velocity object
 *
*/
void ServoCtrl::getPresentVelocity() {
    const char *log;
    auto result = ctrl_interface_->getCtrlInterface()->getVelocity(id_, &present_state_.velocity, &log);
    if (result == false) {
        std::cerr << log << std::endl;
    }
}

/**
 * @brief Get the Present Load object
 *
*/
void ServoCtrl::getPresentLoad() {
    const char *log;
    int32_t data;
    auto result = ctrl_interface_->getCtrlInterface()->itemRead(id_, "Present_Load", &data, &log);
    if (result == false) {
        std::cerr << log << std::endl;
    }
}

/**
 * @brief Get the Present Voltage object
 *
*/
void ServoCtrl::getPresentVoltage() {
    const char *log;
    int32_t data;
    auto result = ctrl_interface_->getCtrlInterface()->itemRead(id_, "Present_Voltage", &data, &log);
    if(result == false) {
        std::cerr << log << std::endl;
    }
    present_state_.voltage = (float)data / 10.0f;
}

/**
 * @brief Get the Present Temperature object
 *
*/
void ServoCtrl::getPresentTemperature() {
    const char *log;
    int32_t data;
    auto result = ctrl_interface_->getCtrlInterface()->itemRead(id_, "Present_Temperature", &data, &log);
    if (result == false) {
        std::cerr << log << std::endl;
    }

    present_state_.temperature = (float)data;
}

void ServoCtrl::ping() {
    const char *log;
    auto result = ctrl_interface_->getCtrlInterface()->ping(id_, &log);
    if (result == false) {
        std::cerr << log << std::endl;
    }
}