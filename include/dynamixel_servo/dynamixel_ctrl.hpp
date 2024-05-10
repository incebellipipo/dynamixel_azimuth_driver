#pragma once

#include "chrono"
#include "functional"
#include "memory"
#include "string"
#include "vector"
#include "memory"

#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include "std_msgs/msg/string.hpp"


class DynamixelCtrl {
private:

    std::string port_;

    int baudrate_;

    std::vector<int> dxl_id_list_;

    std::shared_ptr<DynamixelWorkbench> dxl_wb_;

public:
    DynamixelCtrl() = default;

    void init(std::string port, int baudrate);

    void scanDevices();

    auto getDeviceList() -> decltype(dxl_id_list_);

    auto getCtrlInterface() -> decltype(dxl_wb_);


};

typedef struct  {
    float position;
    float velocity;
    float load;
    float voltage;
    float temperature;
} servo_state_t;

class ServoCtrl {
private:

    std::shared_ptr<DynamixelCtrl> ctrl_interface_;

    int id_;

    servo_state_t present_state_;

    void getPresentPosition();

    void getPresentVelocity();

    void getPresentLoad();

    void getPresentVoltage();

    void getPresentTemperature();


public:

    ServoCtrl(int id, std::shared_ptr<DynamixelCtrl> ctrl_interface);

    void init();

    void enable();

    void update();

    auto getPresentState() -> decltype(present_state_);

    auto getId() -> decltype(id_);

    void setGoalPosition(float goal_position);

    void setGoalVelocity(float goal_velocity);

    void setTorque(bool torque);

    void setWheelMode();

    void setJointMode();

    void ping();

};