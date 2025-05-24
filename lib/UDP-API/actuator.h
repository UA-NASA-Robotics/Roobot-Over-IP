#ifndef ACTUATOR_API_H
#define ACTUATOR_API_H

#include "packetTypes.h"

namespace ActuatorConstants {
namespace MaskConstants {
constexpr uint16_t SET_MASK = 0x0000;   // Set mask
constexpr uint16_t GET_MASK = 0x8000;   // Get mask
constexpr uint8_t LENGTH = 0x00;        // Length operations
constexpr uint8_t VELOCITY = 0x10;      // Velocity operations
constexpr uint8_t HOMING = 0x20;        // Homing operations
constexpr uint8_t CONTROL_FLOW = 0x30;  // Control flow operations
constexpr uint8_t STATE_FLOW = 0x04;    // State flow operations

}  // namespace MaskConstants

/*--------- Action Codes ----------------*/
constexpr actionConstant SET_RELATIVE_LENGTH =
    MaskConstants::SET_MASK | MaskConstants::LENGTH | 0x0;

constexpr actionConstant SET_ABSOLUTE_LENGTH =
    MaskConstants::SET_MASK | MaskConstants::LENGTH | 0x1;

constexpr actionConstant GET_TARGET_LENGTH = MaskConstants::GET_MASK | MaskConstants::LENGTH | 0x2;

constexpr actionConstant GET_CURRENT_LENGTH = MaskConstants::GET_MASK | MaskConstants::LENGTH | 0x3;

constexpr actionConstant SET_VELOCITY = MaskConstants::SET_MASK | MaskConstants::VELOCITY | 0x0;

constexpr actionConstant GET_TARGET_VELOCITY =
    MaskConstants::GET_MASK | MaskConstants::VELOCITY | 0x1;

constexpr actionConstant GET_CURRENT_VELOCITY =
    MaskConstants::GET_MASK | MaskConstants::VELOCITY | 0x2;

constexpr actionConstant GET_LAST_HOME = MaskConstants::GET_MASK | MaskConstants::HOMING | 0x0;

constexpr actionConstant SET_CONTROL = MaskConstants::SET_MASK | MaskConstants::CONTROL_FLOW | 0x0;

constexpr actionConstant GET_CONTROL = MaskConstants::GET_MASK | MaskConstants::CONTROL_FLOW | 0x1;

constexpr actionConstant SET_SPEED_PID = MaskConstants::SET_MASK | MaskConstants::STATE_FLOW | 0x0;

constexpr actionConstant SET_LENGTH_PID = MaskConstants::SET_MASK | MaskConstants::STATE_FLOW | 0x1;

constexpr actionConstant GET_SPEED_PID = MaskConstants::GET_MASK | MaskConstants::STATE_FLOW | 0x2;

constexpr actionConstant GET_LENGTH_PID = MaskConstants::GET_MASK | MaskConstants::STATE_FLOW | 0x3;

//------ Set Control Mode Constants ------
constexpr payloadConstant LENGTH_MODE = 0b00000000;    // Length mode
constexpr payloadConstant VELOCITY_MODE = 0b00000001;  // Velocity mode

}  // namespace ActuatorConstants

#endif  // ACTUATOR_API_H