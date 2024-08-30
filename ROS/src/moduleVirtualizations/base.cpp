#include "base.h"

void BaseModule::debugLog(std::string message) { RCLCPP_INFO(this->get_logger(), message.c_str()); }

uint8_t BaseModule::getOctet() { return _moduleOctet; }

std::string BaseModule::getAlias() { return _moduleAlias; }