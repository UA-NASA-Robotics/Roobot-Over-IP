#include "../include/oDriveError.h"

bool oDriveError::errorIsOperable(uint32_t error) {
    switch (error) {
        case ODRIVE_ERROR_NONE:
            return true;
            break;
        case ODRIVE_ERROR_INITIALIZING:
            return false;
            break;
        case ODRIVE_ERROR_SYSTEM_LEVEL:
            return false;
            break;
        case ODRIVE_ERROR_TIMING_ERROR:
            return false;
            break;
        case ODRIVE_ERROR_MISSING_ESTIMATE:
            return false;
            break;
        case ODRIVE_ERROR_BAD_CONFIG:
            return false;
            break;
        case ODRIVE_ERROR_DRV_FAULT:
            return false;
            break;
        case ODRIVE_ERROR_MISSING_INPUT:
            return false;
            break;
        case ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE:
            return false;
            break;
        case ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE:
            return false;
            break;
        case ODRIVE_ERROR_DC_BUS_OVER_CURRENT:
            return false;
            break;
        case ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT:
            return false;
            break;
        case ODRIVE_ERROR_CURRENT_LIMIT_VIOLATION:
            return false;
            break;
        case ODRIVE_ERROR_MOTOR_OVER_TEMP:
            return false;
            break;
        case ODRIVE_ERROR_INVERTER_OVER_TEMP:
            return false;
            break;
        case ODRIVE_ERROR_VELOCITY_LIMIT_VIOLATION:
            return false;
            break;
        case ODRIVE_ERROR_POSITION_LIMIT_VIOLATION:
            return false;
            break;
        case ODRIVE_ERROR_WATCHDOG_TIMER_EXPIRED:
            return false;
            break;
        case ODRIVE_ERROR_ESTOP_REQUESTED:
            return false;
            break;
        case ODRIVE_ERROR_SPINOUT_DETECTED:
            return false;
            break;
        case ODRIVE_ERROR_BRAKE_RESISTOR_DISARMED:
            return false;
            break;
        case ODRIVE_ERROR_THERMISTOR_DISCONNECTED:
            return true;
            break;
        case ODRIVE_ERROR_CALIBRATION_ERROR:
            return false;
            break;

        default:
            return false;
            break;
    }
}

bool oDriveError::errorShouldAutoClear(uint32_t error) {
    switch (error) {
        case ODRIVE_ERROR_NONE:
            return true;
            break;
        case ODRIVE_ERROR_INITIALIZING:
            return false;
            break;
        case ODRIVE_ERROR_SYSTEM_LEVEL:
            return false;
            break;
        case ODRIVE_ERROR_TIMING_ERROR:
            return false;
            break;
        case ODRIVE_ERROR_MISSING_ESTIMATE:
            return false;
            break;
        case ODRIVE_ERROR_BAD_CONFIG:
            return false;
            break;
        case ODRIVE_ERROR_DRV_FAULT:
            return false;
            break;
        case ODRIVE_ERROR_MISSING_INPUT:
            return false;
            break;
        case ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE:
            return true;
            break;
        case ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE:
            return false;
            break;
        case ODRIVE_ERROR_DC_BUS_OVER_CURRENT:
            return true;
            break;
        case ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT:
            return true;
            break;
        case ODRIVE_ERROR_CURRENT_LIMIT_VIOLATION:
            return true;
            break;
        case ODRIVE_ERROR_MOTOR_OVER_TEMP:
            return false;
            break;
        case ODRIVE_ERROR_INVERTER_OVER_TEMP:
            return false;
            break;
        case ODRIVE_ERROR_VELOCITY_LIMIT_VIOLATION:
            return true;
            break;
        case ODRIVE_ERROR_POSITION_LIMIT_VIOLATION:
            return true;
            break;
        case ODRIVE_ERROR_WATCHDOG_TIMER_EXPIRED:
            return true;
            break;
        case ODRIVE_ERROR_ESTOP_REQUESTED:
            return false;
            break;
        case ODRIVE_ERROR_SPINOUT_DETECTED:
            return false;
            break;
        case ODRIVE_ERROR_BRAKE_RESISTOR_DISARMED:
            return true;
            break;
        case ODRIVE_ERROR_THERMISTOR_DISCONNECTED:
            return true;
            break;
        case ODRIVE_ERROR_CALIBRATION_ERROR:
            return false;
            break;

        default:
            return false;
            break;
    }
}