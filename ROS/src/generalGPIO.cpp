#include "generalGPIO.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GeneralGPIO>());
    rclcpp::shutdown();
    return 0;
}