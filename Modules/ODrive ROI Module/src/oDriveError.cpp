#include "../include/oDriveError.h"

bool oDriveError::errorIsOperable(uint32_t error) {
    switch (error) {
        case ODRIVE_ERROR_NONE:
        case ODRIVE_ERROR_THERMISTOR_DISCONNECTED:
            return true;
            break;
        case ODRIVE_ERROR_INITIALIZING:
        case ODRIVE_ERROR_SYSTEM_LEVEL:
        case ODRIVE_ERROR_TIMING_ERROR:
        case ODRIVE_ERROR_MISSING_ESTIMATE:
        case ODRIVE_ERROR_BAD_CONFIG:
        case ODRIVE_ERROR_DRV_FAULT:
        case ODRIVE_ERROR_MISSING_INPUT:
        case ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE:
        case ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE:
        case ODRIVE_ERROR_DC_BUS_OVER_CURRENT:
        case ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT:
        case ODRIVE_ERROR_CURRENT_LIMIT_VIOLATION:
        case ODRIVE_ERROR_MOTOR_OVER_TEMP:
        case ODRIVE_ERROR_INVERTER_OVER_TEMP:
        case ODRIVE_ERROR_VELOCITY_LIMIT_VIOLATION:
        case ODRIVE_ERROR_POSITION_LIMIT_VIOLATION:
        case ODRIVE_ERROR_WATCHDOG_TIMER_EXPIRED:
        case ODRIVE_ERROR_ESTOP_REQUESTED:
        case ODRIVE_ERROR_SPINOUT_DETECTED:
        case ODRIVE_ERROR_BRAKE_RESISTOR_DISARMED:
        case ODRIVE_ERROR_CALIBRATION_ERROR:
        default:
            return false;
            break;
    }
}

bool oDriveError::errorShouldAutoClear(uint32_t error) {
    switch (error) {
        case ODRIVE_ERROR_NONE:
        case ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE:
        case ODRIVE_ERROR_DC_BUS_OVER_CURRENT:
        case ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT:
        case ODRIVE_ERROR_CURRENT_LIMIT_VIOLATION:
        case ODRIVE_ERROR_VELOCITY_LIMIT_VIOLATION:
        case ODRIVE_ERROR_POSITION_LIMIT_VIOLATION:
        case ODRIVE_ERROR_WATCHDOG_TIMER_EXPIRED:
        case ODRIVE_ERROR_BRAKE_RESISTOR_DISARMED:
        case ODRIVE_ERROR_THERMISTOR_DISCONNECTED:
            return true;
            break;
        case ODRIVE_ERROR_INITIALIZING:
        case ODRIVE_ERROR_SYSTEM_LEVEL:
        case ODRIVE_ERROR_TIMING_ERROR:
        case ODRIVE_ERROR_MISSING_ESTIMATE:
        case ODRIVE_ERROR_BAD_CONFIG:
        case ODRIVE_ERROR_DRV_FAULT:
        case ODRIVE_ERROR_MISSING_INPUT:
        case ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE:
        case ODRIVE_ERROR_MOTOR_OVER_TEMP:
        case ODRIVE_ERROR_INVERTER_OVER_TEMP:
        case ODRIVE_ERROR_ESTOP_REQUESTED:
        case ODRIVE_ERROR_SPINOUT_DETECTED:
        case ODRIVE_ERROR_CALIBRATION_ERROR:
        default:
            return false;
            break;
    }
}