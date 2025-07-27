#include "ODrive.h"

#define __min(a, b) (((a) < (b)) ? (a) : (b))  // idk why i had to define this myself

//-------- Macro Function Declarations -------//
#define __stateMessage(function_name, package_name, message_type, message_val_1, val_1, \
                       message_val_2, val_2, publisher)                                 \
    void ODriveModule::function_name() {                                                \
        /* Publishes variables based on request */                                      \
        auto message = package_name::msg::message_type();                               \
        message.message_val_1 = val_1;                                                  \
        message.message_val_2 = val_2;                                                  \
        this->publisher->publish(message);                                              \
    }

#define __stateMessage4Arg(function_name, package_name, message_type, message_val_1, val_1, \
                           message_val_2, val_2, message_val3, val3, message_val4, val4,    \
                           publisher)                                                       \
    void ODriveModule::function_name() {                                                    \
        /* Publishes variables based on request */                                          \
        auto message = package_name::msg::message_type();                                   \
        message.message_val_1 = val_1;                                                      \
        message.message_val_2 = val_2;                                                      \
        message.message_val3 = val3;                                                        \
        message.message_val4 = val4;                                                        \
        this->publisher->publish(message);                                                  \
    }

#define __goalHandler(function_name, function_name_str, goal_type)                            \
    rclcpp_action::GoalResponse ODriveModule::function_name(                                  \
        const rclcpp_action::GoalUUID &uuid,                                                  \
        std::shared_ptr<const roi_ros::action::goal_type::Goal> goal) {                       \
        this->debugLog("Received goto " function_name_str " action goal request");            \
                                                                                              \
        if (!this->validateVelTorque(goal->target_joint_state.velocity[0],                    \
                                     goal->target_joint_state.effort[0])) {                   \
            this->debugLog("Invalid velocity or torque feedforward");                         \
            return rclcpp_action::GoalResponse::REJECT;                                       \
        }                                                                                     \
        if (this->_healthData._module_error) {                                                \
            this->debugLog("Module error. Rejecting goal");                                   \
            return rclcpp_action::GoalResponse::REJECT;                                       \
        }                                                                                     \
                                                                                              \
        (void)uuid; /* supress unused variable warning */                                     \
        (void)goal; /* supress unused variable warning */                                     \
                                                                                              \
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; /* doesn't matter the goal */ \
    }

#define __gotoServiceHandler(function_name, request_type, request_type_str, velocity,              \
                             feed_forward, calling_function)                                       \
    void ODriveModule::function_name(const roi_ros::srv::request_type::Request::SharedPtr request, \
                                     roi_ros::srv::request_type::Response::SharedPtr response) {   \
        this->debugLog("Received " request_type_str " service request");                           \
        /* Handle the goto (relative) position request */                                          \
        if (!this->validateVelTorque(velocity, feed_forward)) {                                    \
            this->debugLog("Invalid velocity or torque feedforward");                              \
            response->success = false;                                                             \
            return;                                                                                \
        }                                                                                          \
                                                                                                   \
        calling_function;                                                                          \
                                                                                                   \
        response->success = !_healthData._module_error;                                            \
                                                                                                   \
        this->debugLog(request_type_str " service request handled");                               \
    }

#define __actionCancelHandle(function_name, function_name_str, goal_handle_type)                  \
    rclcpp_action::CancelResponse ODriveModule::function_name(                                    \
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<roi_ros::action::goal_handle_type>> \
            goalHandle) {                                                                         \
        this->debugLog("Received goto " function_name_str " action cancel request");              \
                                                                                                  \
        this->sendSetVelocityPacket(0, 0); /* stop the motor */                                   \
                                                                                                  \
        (void)goalHandle; /* Unused variable :( */                                                \
                          /* Not loved */                                                         \
        return rclcpp_action::CancelResponse::ACCEPT;                                             \
    }

#define __actionExecuteHandler(function_name, function_name_str, goal_type, position_condition, \
                               position_update)                                                 \
    void ODriveModule::function_name(                                                           \
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<roi_ros::action::goal_type>>      \
            goalHandle) {                                                                       \
        this->debugLog("Executing goto " function_name_str " action goal loop");                \
        const auto goal = goalHandle->get_goal(); /* get the goal */                            \
        /* outputs of the action */                                                             \
        auto feedback = std::make_shared<roi_ros::action::goal_type::Feedback>();               \
        auto result = std::make_shared<roi_ros::action::goal_type::Result>();                   \
                                                                                                \
        while (rclcpp::ok() && (this->_velocity > 0.5 || /* check if moving at a good speed */  \
                                position_condition)) {   /* check for goal achieved */          \
            feedback->current_joint_state.position[0] = position_update; /* Update feedback */  \
            feedback->current_joint_state.velocity[0] = this->_velocity;                        \
            goalHandle->publish_feedback(feedback); /* publish feedback */                      \
                                                                                                \
            std::this_thread::sleep_for(std::chrono::milliseconds(                              \
                WatchdogConstants::MAINTAIN_SLEEP_TIME)); /* Wait for feedback to be read*/     \
                                                                                                \
            if (goalHandle->is_canceling() || /* Check to see if canceled */                    \
                this->_healthData._module_error) {                                              \
                goalHandle->canceled(result); /* If so, stop */                                 \
                this->debugLog("Goto " function_name_str " action goal canceled");              \
                return;                                                                         \
            }                                                                                   \
        }                                                                                       \
                                                                                                \
        result->success = true; /* Happy function :) */                                         \
        goalHandle->succeed(result);                                                            \
        this->debugLog("Goto " function_name_str " action goal succeeded");                     \
    }

#define __actionAcceptHandler(function_name, function_name_str, goal_type, goto_function,  \
                              execution_handler)                                           \
    void ODriveModule::function_name(                                                      \
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<roi_ros::action::goal_type>> \
            goalHandle) {                                                                  \
        this->debugLog("Received goto " function_name_str                                  \
                       " action accepted request. Spinning up execution monitor thread");  \
                                                                                           \
        this->_relativeStartPosition = this->_position;                                    \
                                                                                           \
        /* Don't wait for the thread, just go now! */                                      \
        this->goto_function(goalHandle->get_goal()->target_joint_state.position[0],        \
                            goalHandle->get_goal()->target_joint_state.velocity[0],        \
                            goalHandle->get_goal()->target_joint_state.effort[0]);         \
                                                                                           \
        std::thread(&ODriveModule::execution_handler, this, goalHandle).detach();          \
    }

//-------- Macro Snippet Declarations -------//
#define __responseCallbackGetVariable(state, variable, data, publisher) \
    case ODriveConstants::MaskConstants::state:                         \
        variable = data;                                                \
        publisher;                                                      \
        break;

#define __responseCallbackSetVariable(state, state_str)                \
    case ODriveConstants::MaskConstants::state:                        \
        this->debugLog(state_str " set");                              \
        if (!data[0]) {                                                \
            this->debugLog(state_str " set failure");                  \
            _healthData._module_error_message = state_str " failure."; \
            this->publishHealthMessage();                              \
        }                                                              \
        break;

#define __oDriveErrorCase(state, message) \
    case ODriveConstants::state:          \
        return message;                   \
        break;

#define __setModeAndSend(actionCode, data)            \
    ROIPackets::Packet packet = ROIPackets::Packet(); \
    packet.setClientAddressOctet(this->getOctet());   \
    packet.setActionCode(actionCode);                 \
    packet.setData(data);                             \
    this->sendGeneralPacket(packet);

//-------- PRIVATE METHODS --------//
void ODriveModule::maintainState() {
    // Maintain the state of the ODrive module

    // Loop to maintain the state
    static uint8_t checkResetCounter = 0;
    // Check if the module has been reset, once every 128 loops (128*50ms = 6.4s)
    if (checkResetCounter == 0) {
        // Issues a status report request. The callback will handle the response.
        // If the callback receives a BLANK_STATE status, it will push the current state to the
        // module.
        ROIPackets::sysAdminPacket statusPacket = ROIPackets::sysAdminPacket();
        statusPacket.setAdminMetaData(sysAdminConstants::NO_CHAIN_META);
        statusPacket.setActionCode(sysAdminConstants::STATUS_REPORT);

        this->sendSysadminPacket(statusPacket);
        checkResetCounter = 128;
    }
    checkResetCounter--;  // Increment the check reset counter

    // this->debugLog("Maintaining state");
    //  Loop through all of the readable values and request their values
    ROIPackets::Packet readPacket = ROIPackets::Packet();
    readPacket.setActionCode(ODriveConstants::GET_ALL);
    this->sendGeneralPacket(readPacket);
}

void ODriveModule::responseCallback(const roi_ros::msg::SerializedPacket response) {
    // Handle the response from the transport agent
    // this->debugLog("Received response from transport agent");

    // Parse the response packet
    ROIPackets::Packet packet = ROIPackets::Packet();
    uint8_t serializedData[ROIConstants::ROI_MAX_PACKET_SIZE];
    this->unpackVectorToArray(response.data, serializedData,
                              __min(response.length, ROIConstants::ROI_MAX_PACKET_SIZE));
    if (!packet.importPacket(serializedData, response.length) &&
        !moduleNodeConstants::IGNORE_MALFORMED_PACKETS) {
        // this->debugLog("Failed to import packet");
        return;
    }

    // Handle the response packet
    uint8_t data[ROIConstants::ROI_MAX_PACKET_PAYLOAD];
    packet.getData(data, ROIConstants::ROI_MAX_PACKET_PAYLOAD);
    if (packet.getActionCode() & ODriveConstants::MaskConstants::GETMASK) {
        // Handle the response to a get request
        switch (packet.getActionCode() & !ODriveConstants::MaskConstants::GETMASK) {
            __responseCallbackGetVariable(ControlMode, _controlMode, data[0], (void)"");
            __responseCallbackGetVariable(InputMode, _inputMode, data[0], (void)"");
            __responseCallbackGetVariable(Torque, _inputTorque, floatCast::toFloat(data, 0, 3),
                                          (void)"");
            __responseCallbackGetVariable(PositionSetPoint, _inputPosition,
                                          floatCast::toFloat(data, 0, 3), (void)"");
            __responseCallbackGetVariable(VelocitySetPoint, _inputVelocity,
                                          floatCast::toFloat(data, 0, 3), (void)"");
            __responseCallbackGetVariable(
                Error, _healthData._module_error_message,
                this->oDriveErrorToString(data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]),
                this->publishHealthMessage());
            __responseCallbackGetVariable(Velocity, _velocity, floatCast::toFloat(data, 0, 3),
                                          this->publishStateMessage());
            __responseCallbackGetVariable(Position, _position, floatCast::toFloat(data, 0, 3),
                                          this->publishStateMessage());
            __responseCallbackGetVariable(BusVoltage, _busVoltage, floatCast::toFloat(data, 0, 3),
                                          this->publishPowerMessage());
            __responseCallbackGetVariable(Current, _current, floatCast::toFloat(data, 0, 3),
                                          this->publishPowerMessage());
            __responseCallbackGetVariable(MotorTemperature, _motorTemperature,
                                          floatCast::toFloat(data, 0, 3),
                                          this->publishTemperatureMessage());
            __responseCallbackGetVariable(FETTemperature, _fetTemperature,
                                          floatCast::toFloat(data, 0, 3),
                                          this->publishTemperatureMessage());

            case ODriveConstants::MaskConstants::KinematicFeedback:
                _position = floatCast::toFloat(data, 0, 3);
                _velocity = floatCast::toFloat(data, 4, 7);

                this->publishStateMessage();
                break;

            case ODriveConstants::MaskConstants::all:
                _position = floatCast::toFloat(data, 0, 3);
                _velocity = floatCast::toFloat(data, 4, 7);
                _busVoltage = floatCast::toFloat(data, 8, 11);
                _current = floatCast::toFloat(data, 12, 15);
                _fetTemperature = floatCast::toFloat(data, 16, 19);
                _motorTemperature = floatCast::toFloat(data, 20, 23);

                _errorCode = data[24] << 24 | data[25] << 16 | data[26] << 8 | data[27];
                _healthData._module_error_message = this->oDriveErrorToString(_errorCode);

                this->publishHealthMessage();
                this->publishPowerMessage();
                this->publishTemperatureMessage();
                this->publishStateMessage();

                break;

                // default:debugLog
                // this->debugLog("Unknown get action code received: " +
                //                std::to_string(packet.getActionCode()));
                //    break;
        }
    } else {
        // Handle the response to a set request
        switch (packet.getActionCode() & !ODriveConstants::MaskConstants::GETMASK) {
            __responseCallbackSetVariable(ControlMode, "control mode");
            __responseCallbackSetVariable(InputMode, "input mode");
            __responseCallbackSetVariable(Torque, "torque");
            __responseCallbackSetVariable(PositionSetPoint, "position");
            __responseCallbackSetVariable(VelocitySetPoint, "velocity");
            __responseCallbackSetVariable(PositionRelative, "relative position");
            __responseCallbackSetVariable(Error, "error clear");

            default:
                this->debugLog("Unknown action code received: " +
                               std::to_string(packet.getActionCode()));
                break;
        }
    }

    // this->debugLog("Response handled");
}

void ODriveModule::sendGotoPositionPacket(float position, float velocity_feedforward,
                                          float torque_feedforward) {
    // Set the odrive to position mode if needed to complete request
    if (_controlMode != ODriveConstants::POSITION_MODE) {
        __setModeAndSend(ODriveConstants::SET_CONTROL_MODE, ODriveConstants::POSITION_MODE);

        _controlMode = ODriveConstants::POSITION_MODE;
    }
    if (_inputMode != ODriveConstants::TRAP_TRAJ_MODE ||
        _inputMode != ODriveConstants::POS_FILTER_MODE) {
        __setModeAndSend(ODriveConstants::SET_INPUT_MODE, ODriveConstants::TRAP_TRAJ_MODE);

        _inputMode = ODriveConstants::TRAP_TRAJ_MODE;
    }

    if (torque_feedforward != 0) {  // we have a torque feedforward to contribute
        __setModeAndSend(ODriveConstants::SET_TORQUE, torque_feedforward);
    }
    if (velocity_feedforward != 0) {  // we have a velocity feedforward to contribute
        __setModeAndSend(ODriveConstants::SET_VELOCITY, velocity_feedforward);
    }

    // Send the position set point
    __setModeAndSend(ODriveConstants::SET_POSITION, position);
}

void ODriveModule::sendGotoRelativePositionPacket(float position, float velocity_feedforward,
                                                  float torque_feedforward) {
    // Set the odrive to position mode if needed to complete request
    if (_controlMode != ODriveConstants::POSITION_MODE) {
        __setModeAndSend(ODriveConstants::SET_CONTROL_MODE, ODriveConstants::POSITION_MODE);

        _controlMode = ODriveConstants::POSITION_MODE;
    }
    if (_inputMode != ODriveConstants::TRAP_TRAJ_MODE ||
        _inputMode != ODriveConstants::POS_FILTER_MODE) {
        __setModeAndSend(ODriveConstants::SET_INPUT_MODE, ODriveConstants::TRAP_TRAJ_MODE);

        _inputMode = ODriveConstants::TRAP_TRAJ_MODE;
    }

    if (torque_feedforward != 0) {  // we have a torque feedforward to contribute
        __setModeAndSend(ODriveConstants::SET_TORQUE, torque_feedforward);
    }
    if (velocity_feedforward != 0) {  // we have a velocity feedforward to contribute
        __setModeAndSend(ODriveConstants::SET_VELOCITY, this->radToRev(velocity_feedforward));
    }

    // Send the position set point
    __setModeAndSend(ODriveConstants::SET_RELATIVE_POSITION, this->radToRev(position));

    this->_relativeStartPosition =
        this->_position;  // store the relative start position (we do this here to hack it into the
                          // action server macro... yuck)
}

void ODriveModule::sendSetTorquePacket(float torque) {
    if (_controlMode != ODriveConstants::TORQUE_MODE) {
        __setModeAndSend(ODriveConstants::SET_CONTROL_MODE, ODriveConstants::TORQUE_MODE);

        _controlMode = ODriveConstants::TORQUE_MODE;
    }

    if (_inputMode != ODriveConstants::TORQUE_RAMP_MODE) {
        __setModeAndSend(ODriveConstants::SET_INPUT_MODE, ODriveConstants::TORQUE_RAMP_MODE);

        _inputMode = ODriveConstants::TORQUE_RAMP_MODE;
    }

    __setModeAndSend(ODriveConstants::SET_TORQUE, torque);
}

void ODriveModule::sendSetVelocityPacket(float velocity, float torque_feedforward) {
    if (_controlMode != ODriveConstants::VELOCITY_MODE) {
        __setModeAndSend(ODriveConstants::SET_CONTROL_MODE, ODriveConstants::VELOCITY_MODE);

        _controlMode = ODriveConstants::VELOCITY_MODE;
    }

    if (_inputMode != ODriveConstants::VELOCITY_RAMP_MODE) {
        __setModeAndSend(ODriveConstants::SET_INPUT_MODE, ODriveConstants::VELOCITY_RAMP_MODE);

        _inputMode = ODriveConstants::VELOCITY_RAMP_MODE;
    }

    if (torque_feedforward != 0) {
        __setModeAndSend(ODriveConstants::SET_TORQUE, torque_feedforward);
    }

    __setModeAndSend(ODriveConstants::SET_VELOCITY, this->radToRev(velocity));
}

std::string ODriveModule::oDriveErrorToString(uint32_t errorCode) {
    switch (errorCode) {
        __oDriveErrorCase(ODRIVE_ERROR_BAD_CONFIG, "Bad Configuration, invalid settings values.");
        __oDriveErrorCase(ODRIVE_ERROR_BRAKE_RESISTOR_DISARMED, "Brake resistor disarmed.");
        __oDriveErrorCase(ODRIVE_ERROR_CALIBRATION_ERROR,
                          "Calibration error, manually re-calibrate motor.");
        __oDriveErrorCase(
            ODRIVE_ERROR_CURRENT_LIMIT_VIOLATION,
            "Current limit violation, current limit exceeded. Auto reset attempting.");
        __oDriveErrorCase(ODRIVE_ERROR_DC_BUS_OVER_CURRENT,
                          "DC bus over current, current limit exceeded. Auto reset attempting.");
        __oDriveErrorCase(
            ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT,
            "DC bus over regen current, current limit exceeded. Auto reset attempting.");
        __oDriveErrorCase(
            ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE,
            "DC bus over voltage, reduce allowed regen current. Auto reset attempting.");
        __oDriveErrorCase(ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE,
                          "DC bus under voltage, check power supply.");
        __oDriveErrorCase(ODRIVE_ERROR_DRV_FAULT,
                          "Driver fault, check motor and encoder connections.");
        __oDriveErrorCase(ODRIVE_ERROR_ESTOP_REQUESTED,
                          "E-stop requested, check for e-stop condition.");
        __oDriveErrorCase(ODRIVE_ERROR_INITIALIZING,
                          "Initializing, wait for initialization to complete.");
        __oDriveErrorCase(ODRIVE_ERROR_INVERTER_OVER_TEMP,
                          "Inverter over temperature, reduce load or increase cooling.");
        __oDriveErrorCase(ODRIVE_ERROR_MISSING_ESTIMATE,
                          "Missing estimate, check encoder connection.");
        __oDriveErrorCase(ODRIVE_ERROR_MISSING_INPUT,
                          "Missing input, check network and module firmware.");
        __oDriveErrorCase(ODRIVE_ERROR_MOTOR_OVER_TEMP,
                          "Motor over temperature, reduce load or increase cooling.");
        __oDriveErrorCase(ODRIVE_ERROR_NONE, "");
        __oDriveErrorCase(ODRIVE_ERROR_POSITION_LIMIT_VIOLATION,
                          "Position limit violation, check position limits.");
        __oDriveErrorCase(ODRIVE_ERROR_SPINOUT_DETECTED,
                          "Spinout detected, check motor and encoder connections.");
        __oDriveErrorCase(ODRIVE_ERROR_SYSTEM_LEVEL,
                          "System level error, check ODrive firmware/replace.");
        __oDriveErrorCase(ODRIVE_ERROR_THERMISTOR_DISCONNECTED,
                          "Motor thermistor disconnected, check thermistor connection.");
        __oDriveErrorCase(ODRIVE_ERROR_TIMING_ERROR,
                          "Timing error, check motor and encoder connections.");
        __oDriveErrorCase(
            ODRIVE_ERROR_VELOCITY_LIMIT_VIOLATION,
            "Velocity limit violation, check velocity limits. Auto reset attempting.");
        __oDriveErrorCase(ODRIVE_ERROR_WATCHDOG_TIMER_EXPIRED,
                          "Watchdog timer expired, check module to oDrive connection.");
        default:
            return "Unknown error code.";
            break;
    }
}

bool ODriveModule::validateVelTorque(float velocity, float torque) {
    // Validate the velocity and torque values
    if (abs(velocity) > this->get_parameter("max_velocity").as_double() ||
        abs(torque) > this->get_parameter("max_torque").as_double()) {
        return false;
    }
    return true;
}

float ODriveModule::revToRad(float revs) {
    // Convert revolutions to radians
    return revs * 180 / M_PI;  // 180 / M_PI is the conversion factor from radians to degrees
}

float ODriveModule::radToRev(float radians) {
    // Convert radians to revolutions
    return radians * M_PI / 180;  // M_PI / 180 is the conversion factor from degrees to radians
}

//-------- MACRO METHODS ---------//

__stateMessage(publishPowerMessage, roi_ros, Power, voltage, _busVoltage, current, _current,
               _power_publisher_);

__stateMessage4Arg(publishStateMessage, sensor_msgs, JointState, position,
                   std::vector<double>{this->revToRad(_position)}, velocity,
                   std::vector<double>{this->revToRad(_velocity)}, name,
                   std::vector<std::string>{"odrive.axis0"}, effort, std::vector<double>{_current},
                   _state_publisher_);

__stateMessage(publishFetTemperatureMessage, sensor_msgs, Temperature, temperature,
               (double)_fetTemperature, variance, 0, _fet_temperature_publisher_);

__stateMessage(publishMotorTemperatureMessage, sensor_msgs, Temperature, temperature,
               (double)_motorTemperature, variance, 0, _motor_temperature_publisher_);

void ODriveModule::publishTemperatureMessage() {
    // Publish the temperature message
    this->publishFetTemperatureMessage();
    this->publishMotorTemperatureMessage();
}

__gotoServiceHandler(
    gotoPositionServiceHandler, TargetJointState, "goto position",
    request->target_joint_state.velocity[0], request->target_joint_state.effort[0],
    this->sendGotoPositionPacket(this->radToRev(request->target_joint_state.position[0]),
                                 this->radToRev(request->target_joint_state.velocity[0]),
                                 request->target_joint_state.effort[0]));

__gotoServiceHandler(
    gotoRelativePositionServiceHandler, TargetJointState, "goto relative position",
    request->target_joint_state.velocity[0], request->target_joint_state.effort[0],
    this->sendGotoRelativePositionPacket(this->radToRev(request->target_joint_state.position[0]),
                                         this->radToRev(request->target_joint_state.velocity[0]),
                                         request->target_joint_state.effort[0]));

__gotoServiceHandler(setTorqueServiceHandler, TargetJointState, "set torque", 0,
                     request->target_joint_state.effort[0],
                     this->sendSetTorquePacket(request->target_joint_state.effort[0]));

__gotoServiceHandler(
    setVelocityServiceHandler, TargetJointState, "set velocity",
    request->target_joint_state.velocity[0], request->target_joint_state.effort[0],
    this->sendSetVelocityPacket(this->radToRev(request->target_joint_state.velocity[0]),
                                request->target_joint_state.effort[0]));

__goalHandler(gotoPositionGoalHandler, "position", TargetJointState);
__actionAcceptHandler(gotoPositionAcceptedHandler, "position", TargetJointState,
                      sendGotoPositionPacket, gotoPositionExecuteHandler);

__actionCancelHandle(gotoPositionCancelHandler, "position", TargetJointState);

__actionExecuteHandler(gotoPositionExecuteHandler, "position", TargetJointState,
                       this->_position == this->radToRev(goal->target_joint_state.position[0]),
                       this->revToRad(this->_position));

__goalHandler(gotoRelativePositionGoalHandler, "relative position", TargetJointState);

__actionAcceptHandler(gotoRelativePositionAcceptedHandler, "relative position", TargetJointState,
                      sendGotoRelativePositionPacket, gotoRelativePositionExecuteHandler);

__actionCancelHandle(gotoRelativePositionCancelHandler, "relative position", TargetJointState);

__actionExecuteHandler(gotoRelativePositionExecuteHandler, "relative position", TargetJointState,
                       abs(this->radToRev(goal->target_joint_state.position[0]) - this->_position -
                           this->_relativeStartPosition),
                       this->revToRad(this->_position - this->_relativeStartPosition));

//-------- PUBLIC METHODS --------//

ODriveModule::ODriveModule() : BaseModule("ODriveModule", moduleTypesConstants::O_DRIVE) {
    // Initialize the ODrive module
    // this->debugLog("Initializing ODrive Module");

    // Initialize the ODrive specific parameters
    this->declare_parameter<float>("max_velocity", 565.0);  // rad/s
    this->declare_parameter<float>("max_torque", 2.0);      // nm

    this->_controlMode = ODriveConstants::POSITION_MODE;
    this->_inputMode = ODriveConstants::TRAP_TRAJ_MODE;

    // Initialize the ODrive specific publishers
    this->_power_publisher_ =
        this->create_publisher<roi_ros::msg::Power>("roi_ros/odrv/axis0/power", 10);
    this->_state_publisher_ =
        this->create_publisher<sensor_msgs::msg::JointState>("roi_ros/odrv/axis0/state", 10);
    this->_fet_temperature_publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>(
        "roi_ros/odrv/axis0/fet_temperature", 10);
    this->_motor_temperature_publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>(
        "roi_ros/odrv/axis0/motor_temperature", 10);

    // Initialize the ODrive specific services
    this->_goto_position_service_ = this->create_service<roi_ros::srv::TargetJointState>(
        "roi_ros/odrv/axis0/goto_position",
        std::bind(&ODriveModule::gotoPositionServiceHandler, this, std::placeholders::_1,
                  std::placeholders::_2));
    this->_goto_relative_position_service_ = this->create_service<roi_ros::srv::TargetJointState>(
        "roi_ros/odrv/axis0/goto_relative_position",
        std::bind(&ODriveModule::gotoRelativePositionServiceHandler, this, std::placeholders::_1,
                  std::placeholders::_2));
    this->_set_torque_service_ = this->create_service<roi_ros::srv::TargetJointState>(
        "roi_ros/odrv/axis0/set_torque", std::bind(&ODriveModule::setTorqueServiceHandler, this,
                                                   std::placeholders::_1, std::placeholders::_2));
    this->_set_velocity_service_ = this->create_service<roi_ros::srv::TargetJointState>(
        "roi_ros/odrv/axis0/set_velocity", std::bind(&ODriveModule::setVelocityServiceHandler, this,
                                                     std::placeholders::_1, std::placeholders::_2));

    // Initialize the ODrive specific action servers
    this->_goto_position_action_server_ =
        rclcpp_action::create_server<roi_ros::action::TargetJointState>(
            this->get_node_base_interface(), this->get_node_clock_interface(),
            this->get_node_logging_interface(), this->get_node_waitables_interface(),
            "roi_ros/odrv/axis0/goto_position",
            std::bind(&ODriveModule::gotoPositionGoalHandler, this, std::placeholders::_1,
                      std::placeholders::_2),
            std::bind(&ODriveModule::gotoPositionCancelHandler, this, std::placeholders::_1),
            std::bind(&ODriveModule::gotoPositionAcceptedHandler, this, std::placeholders::_1));
    this->_goto_relative_position_action_server_ =
        rclcpp_action::create_server<roi_ros::action::TargetJointState>(
            this->get_node_base_interface(), this->get_node_clock_interface(),
            this->get_node_logging_interface(), this->get_node_waitables_interface(),
            "roi_ros/odrv/axis0/goto_relative_position",
            std::bind(&ODriveModule::gotoRelativePositionGoalHandler, this, std::placeholders::_1,
                      std::placeholders::_2),
            std::bind(&ODriveModule::gotoRelativePositionCancelHandler, this,
                      std::placeholders::_1),
            std::bind(&ODriveModule::gotoRelativePositionAcceptedHandler, this,
                      std::placeholders::_1));

    this->debugLog("ODrive Module Initialized");

    // Send a status report packet to check if the module is in a blank state
    ROIPackets::sysAdminPacket statusPacket = ROIPackets::sysAdminPacket();
    statusPacket.setAdminMetaData(sysAdminConstants::NO_CHAIN_META);
    statusPacket.setActionCode(sysAdminConstants::STATUS_REPORT);
    statusPacket.setClientAddressOctet(this->getOctet());

    this->sendSysadminPacket(statusPacket);
}

ODriveModule::~ODriveModule() {
    // Destroy the GPIO module
    // this->debugLog("Destroying ODrive Module");
}

bool ODriveModule::pushState() {
    // Push the current stored state of the module to the physical module
    // This is used to recover the state of the module after a hardware reset but Ros node still
    // alive.

    // this->debugLog("Pushing state to ODrive module");

    // Push the control mode
    ROIPackets::Packet packet = ROIPackets::Packet();
    packet.setClientAddressOctet(this->getOctet());
    packet.setActionCode(ODriveConstants::SET_CONTROL_MODE);
    packet.setData(_controlMode);
    this->sendGeneralPacket(packet);

    // Push the input mode
    packet.setActionCode(ODriveConstants::SET_INPUT_MODE);
    packet.setData(_inputMode);
    this->sendGeneralPacket(packet);

    // Push the input torque
    packet.setActionCode(ODriveConstants::SET_TORQUE);
    packet.setData_impFloatCast(_inputTorque);
    this->sendGeneralPacket(packet);

    // Push the input position
    packet.setActionCode(ODriveConstants::SET_POSITION);
    packet.setData_impFloatCast(_inputPosition);
    this->sendGeneralPacket(packet);

    // Push the input velocity
    packet.setActionCode(ODriveConstants::SET_VELOCITY);
    packet.setData_impFloatCast(_inputVelocity);
    this->sendGeneralPacket(packet);

    // this->debugLog("State pushed to ODrive module");

    return true;
}

bool ODriveModule::pullState() {
    // Pull the current state of the module from the physical module. Used to recover a ROS node
    // restart when the module is still running

    // this->debugLog("Pulling state from ODrive module");

    // Request the control mode
    ROIPackets::Packet packet = ROIPackets::Packet();
    packet.setClientAddressOctet(this->getOctet());

    packet.setActionCode(ODriveConstants::GET_CONTROL_MODE);
    this->sendGeneralPacket(packet);

    // Request the input mode
    packet.setActionCode(ODriveConstants::GET_INPUT_MODE);
    this->sendGeneralPacket(packet);

    // Request the input torque
    packet.setActionCode(ODriveConstants::GET_TORQUE_SETPOINT);
    this->sendGeneralPacket(packet);

    // Request the input position
    packet.setActionCode(ODriveConstants::GET_POSITION_SETPOINT);
    this->sendGeneralPacket(packet);

    // Request the input velocity
    packet.setActionCode(ODriveConstants::GET_VELOCITY_SETPOINT);
    this->sendGeneralPacket(packet);

    // Get all the non-state data
    packet.setActionCode(ODriveConstants::GET_ALL);
    this->sendGeneralPacket(packet);

    // this->debugLog("State pulled from ODrive module");

    return true;
}

#ifdef ODRIVE_MODULE
// We may wish to include and extend upon the oDrive module in the future, in which case we do not
// wish to execute this main. Disable by not defining ODRIVE_MODULE in CMAKELists.txt
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ODriveModule>());
    rclcpp::shutdown();
    return 0;
}
#endif  // ODRIVE_MODULE
        // python users fear the chad 700 line .cpp file