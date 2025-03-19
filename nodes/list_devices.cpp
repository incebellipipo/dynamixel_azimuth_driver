#include "dynamixel_servo/dynamixel_ctrl.hpp"
#include "iostream"

int main(int argc, char *argv[]) {

    auto dynamixel_ctrl = std::make_shared<DynamixelCtrl>();

    if(argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <port> <baudrate>" << std::endl;
        return 1;
    }

    std::string port(argv[1]);
    int baudrate = std::stoi(argv[2]);

    std::cout << "Initializing Dynamixel controller on port: " << port << " with baudrate: " << baudrate << std::endl;

    dynamixel_ctrl->init(
        port,
        baudrate
    );

    std::cout << "Scanning for devices..." << std::endl;

    dynamixel_ctrl->scanDevices();

    std::cout << "Found " << dynamixel_ctrl->getDeviceList().size() << " devices." << std::endl;

    for(auto i : dynamixel_ctrl->getDeviceList()){
        std::cout << "Found device with id: " << i << std::endl;
    }


    return 0;
}