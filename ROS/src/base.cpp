#include "base.h"

void BaseModule::debugLog(std::string message) { RCLCPP_INFO(this->get_logger(), message.c_str()); }

uint8_t BaseModule::getOctet() { return this->get_parameter("module_octet").as_int(); }

std::string BaseModule::getAlias() { return this->get_parameter("module_alias").as_string(); }

int main(int argc, char *argv[]) {
    std::cout << "Hello, world!" << std::endl;
    return 0;
}