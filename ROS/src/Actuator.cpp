#include "Actuator.h"

#define __min(a, b) (((a) < (b)) ? (a) : (b))  // idk why i had to define this myself

#define __gotoServiceHandler(function_name, request_type, request_type_str, position, velocity, \
                             sub_device_id, calling_function)                                   \
    void ActuatorModule::function_name(                                                         \
        const roi_ros::srv::request_type::Request::SharedPtr request,                           \
        roi_ros::srv::request_type::Response::SharedPtr response) {                             \
        this->debugLog("Received " request_type_str " service request");                        \
        /* Handle the goto (relative) position request */                                       \
        if (!this->validateInput(position, velocity, sub_device_id)) {                          \
            this->debugLog("Invalid velocity or torque feedforward");                           \
            response->success = false;                                                          \
            return;                                                                             \
        }                                                                                       \
                                                                                                \
        calling_function;                                                                       \
                                                                                                \
        response->success = !_healthData._module_error;                                         \
                                                                                                \
        this->debugLog(request_type_str " service request handled");                            \
    }

#define __actionCancelHandle(function_name, function_name_str, goal_handle_type)                  \
    rclcpp_action::CancelResponse ActuatorModule::function_name(                                  \
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<roi_ros::action::goal_handle_type>> \
            goalHandle) {                                                                         \
        this->debugLog("Received goto " function_name_str " action cancel request");              \
                                                                                                  \
        this->sendSetVelocityPacket(0, goalHandle->get_goal()->sub_device_id);                    \
                                                                                                  \
        return rclcpp_action::CancelResponse::ACCEPT;                                             \
    }

#define __actionExecuteHandler(function_name, function_name_str, goal_type, position_condition, \
                               position_update)                                                 \
    void ActuatorModule::function_name(                                                         \
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<roi_ros::action::goal_type>>      \
            goalHandle) {                                                                       \
        this->debugLog("Executing goto " function_name_str " action goal loop");                \
        const auto goal = goalHandle->get_goal(); /* get the goal */                            \
        /* outputs of the action */                                                             \
        auto feedback = std::make_shared<roi_ros::action::goal_type::Feedback>();               \
        auto result = std::make_shared<roi_ros::action::goal_type::Result>();                   \
                                                                                                \
        while (rclcpp::ok() && (this->_velocities[goal->sub_device_id] >                        \
                                    0.5 ||             /* check if moving at a good speed */    \
                                position_condition)) { /* check for goal achieved */            \
            feedback->current_joint_state.position[0] = position_update; /* Update feedback */  \
            feedback->current_joint_state.velocity[0] = this->_velocities[goal->sub_device_id]; \
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
    void ActuatorModule::function_name(                                                    \
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<roi_ros::action::goal_type>> \
            goalHandle) {                                                                  \
        auto goal = goalHandle->get_goal();                                                \
        this->debugLog("Received goto " function_name_str                                  \
                       " action accepted request. Spinning up execution monitor thread");  \
                                                                                           \
        this->_relativeStartPositions[goal->sub_device_id] =                               \
            this->_positions[goal->sub_device_id];                                         \
                                                                                           \
        /* Don't wait for the thread, just go now! */                                      \
        this->goto_function(goal->target_joint_state.position[0],                          \
                            goal->target_joint_state.velocity[0], goal->sub_device_id);    \
                                                                                           \
        std::thread(&ActuatorModule::execution_handler, this, goalHandle).detach();        \
    }

#define __responseCallbackGetVariable(state, variable, data, publisher)                        \
    case ActuatorConstants::MaskConstants::GET_MASK | ActuatorConstants::MaskConstants::state: \
        variable = data;                                                                       \
        publisher;                                                                             \
        break;

#define __responseCallbackSetVariable(state, state_str)                                        \
    case ActuatorConstants::MaskConstants::SET_MASK | ActuatorConstants::MaskConstants::state: \
        this->debugLog(state_str " set");                                                      \
        if (!data[0]) {                                                                        \
            this->debugLog(state_str " set failure");                                          \
            _healthData._module_error_message = state_str " failure.";                         \
            this->publishHealthMessage();                                                      \
        }                                                                                      \
        break;

#define __matchPositionCopy(vect1, vect2)       \
    for (size_t i = 0; i < vect1.size(); i++) { \
        if (i < vect2.size()) {                 \
            vect2[i] = vect1[i];                \
        } else {                                \
            vect2.push_back(vect1[i]);          \
        }                                       \
    }

//-------- PRIVATE METHODS --------//
void ActuatorModule::maintainState() {
    // Maintain the state of the Actuator module

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
    ROIPackets::Packet readVelPacket = ROIPackets::Packet();
    ROIPackets::Packet readLengthPacket = ROIPackets::Packet();
    readVelPacket.setClientAddressOctet(this->getOctet());
    readLengthPacket.setClientAddressOctet(this->getOctet());
    readVelPacket.setActionCode(ActuatorConstants::GET_CURRENT_VELOCITY);
    readLengthPacket.setActionCode(ActuatorConstants::GET_CURRENT_LENGTH);

    for (uint8_t i = 0; i < this->_controlModes.size(); i++) {
        readVelPacket.setSubDeviceID(i);
        readLengthPacket.setSubDeviceID(i);

        // Request the current velocity
        this->sendGeneralPacket(readVelPacket);
        // Request the current length
        this->sendGeneralPacket(readLengthPacket);
    }
}

void ActuatorModule::responseCallback(const roi_ros::msg::SerializedPacket response) {
    // Handle the response from the transport agent
    this->debugLog("Received response from transport agent");

    // Parse the response packet
    ROIPackets::Packet packet = ROIPackets::Packet();
    uint8_t serializedData[ROIConstants::ROI_MAX_PACKET_SIZE];
    this->unpackVectorToArray(response.data, serializedData,
                              __min(response.length, ROIConstants::ROI_MAX_PACKET_SIZE));
    if (!packet.importPacket(serializedData, response.length) &&
        !moduleNodeConstants::IGNORE_MALFORMED_PACKETS) {
        this->debugLog("Failed to import packet");
        return;
    }

    // Handle the response packet
    uint8_t data[ROIConstants::ROI_MAX_PACKET_PAYLOAD];
    packet.getData(data, ROIConstants::ROI_MAX_PACKET_PAYLOAD);
    uint16_t subDeviceID = packet.getSubDeviceID();
    // Handle the response to a get request
    switch (packet.getActionCode()) {
        __responseCallbackGetVariable(CONTROL_FLOW | 0x1, _controlModes[subDeviceID], data[0],
                                      (void)"");
        __responseCallbackGetVariable(LENGTH | 0x2, _inputPositions[subDeviceID],
                                      ((uint16_t)data[0] << 8) + data[1], (void)"");
        __responseCallbackGetVariable(LENGTH | 0x3, _positions[subDeviceID],
                                      ((uint16_t)data[0] << 8) + data[1],
                                      _publishStateMessages[subDeviceID]());
        __responseCallbackGetVariable(VELOCITY | 0x1, _inputVelocities[subDeviceID],
                                      ((uint16_t)data[0] << 8) + data[1], (void)"");
        __responseCallbackGetVariable(VELOCITY | 0x2, _velocities[subDeviceID],
                                      ((uint16_t)data[0] << 8) + data[1],
                                      _publishStateMessages[subDeviceID]());
        __responseCallbackGetVariable(HOMING, _homeElapsedTimes[subDeviceID],
                                      ((uint32_t)data[0] << 24) + ((uint32_t)data[1] << 16) +
                                          ((uint32_t)data[2] << 8) + data[3],
                                      _publishHomeElapsedTimeMessages[subDeviceID]());
        case ActuatorConstants::MaskConstants::GET_MASK |
            ActuatorConstants::MaskConstants::STATE_FLOW | 0x2: {
            std::vector<double> currentParam =
                this->get_parameter("velocity_pid").as_double_array();
            if (currentParam.size() < (unsigned)(subDeviceID * 3 + 3)) {
                throw std::runtime_error(
                    "Velocity PID parameter size is smaller than expected for subDeviceID: " +
                    std::to_string(subDeviceID));
            }

            // p
            currentParam[subDeviceID * 3] = floatCast::toFloat(data, 0, 3);
            // i
            currentParam[subDeviceID * 3 + 1] = floatCast::toFloat(data, 4, 7);
            // d
            currentParam[subDeviceID * 3 + 2] = floatCast::toFloat(data, 8, 11);

            this->set_parameter(rclcpp::Parameter("velocity_pid", currentParam));
        } break;

        case ActuatorConstants::MaskConstants::GET_MASK |
            ActuatorConstants::MaskConstants::STATE_FLOW | 0x3: {
            std::vector<double> currentParam =
                this->get_parameter("position_pid").as_double_array();

            if (currentParam.size() < (unsigned)(subDeviceID * 3 + 3)) {
                throw std::runtime_error(
                    "Position PID parameter size is smaller than expected for subDeviceID: " +
                    std::to_string(subDeviceID));
            }

            // p
            currentParam[subDeviceID * 3] = floatCast::toFloat(data, 0, 3);
            // i
            currentParam[subDeviceID * 3 + 1] = floatCast::toFloat(data, 4, 7);
            // d
            currentParam[subDeviceID * 3 + 2] = floatCast::toFloat(data, 8, 11);

            this->set_parameter(rclcpp::Parameter("position_pid", currentParam));
        } break;

            __responseCallbackSetVariable(CONTROL_FLOW | 0x0, "Control Mode");
            __responseCallbackSetVariable(LENGTH | 0x0, "Relative Length");
            __responseCallbackSetVariable(LENGTH | 0x1, "Absolute Length");
            __responseCallbackSetVariable(VELOCITY | 0x0, "Velocity");
            __responseCallbackSetVariable(STATE_FLOW | 0x0, "Speed PID");
            __responseCallbackSetVariable(STATE_FLOW | 0x1, "Position PID");

        default:
            this->debugLog("Unknown get action code received: " +
                           std::to_string(packet.getActionCode()));
            break;
    }

    // this->debugLog("Response handled");
}

//-------- SERVICE METHODS --------//

__gotoServiceHandler(_goto_position_service_callback_, TargetJointState, "Goto Position",
                     request->target_joint_state.position[0],
                     request->target_joint_state.velocity[0], request->sub_device_id,
                     this->sendGotoAbsolutePositionPacket(request->target_joint_state.position[0],
                                                          request->target_joint_state.velocity[0],
                                                          request->sub_device_id));

__gotoServiceHandler(_goto_relative_position_service_callback_, TargetJointState,
                     "Goto Relative Position", request->target_joint_state.position[0],
                     request->target_joint_state.velocity[0], request->sub_device_id,
                     this->sendGotoRelativePositionPacket(request->target_joint_state.position[0],
                                                          request->target_joint_state.velocity[0],
                                                          request->sub_device_id));

__gotoServiceHandler(_set_velocity_service_callback_, TargetJointState, "Set Velocity",
                     request->target_joint_state.velocity[0], 0, request->sub_device_id,
                     this->sendSetVelocityPacket(request->target_joint_state.velocity[0],
                                                 request->sub_device_id));

//-------- ACTION METHODS --------//

__actionCancelHandle(_goto_position_cancel_handler_, "Position", TargetJointState);
__actionCancelHandle(_goto_relative_position_cancel_handler_, "Relative Position",
                     TargetJointState);

__actionExecuteHandler(_goto_position_execute_handler_, "Position", TargetJointState,
                       _positions[goal->sub_device_id] == goal->target_joint_state.position[0],
                       _positions[goal->sub_device_id]);

__actionExecuteHandler(
    _goto_relative_position_execute_handler_, "Relative Position", TargetJointState,
    _positions[goal->sub_device_id] ==
        _relativeStartPositions[goal->sub_device_id] + goal->target_joint_state.position[0],
    _positions[goal->sub_device_id] - _relativeStartPositions[goal->sub_device_id]);

__actionAcceptHandler(_goto_position_accepted_handler_, "Position", TargetJointState,
                      sendGotoAbsolutePositionPacket, _goto_position_execute_handler_);

__actionAcceptHandler(_goto_relative_position_accepted_handler_, "Relative Position",
                      TargetJointState, sendGotoRelativePositionPacket,
                      _goto_relative_position_execute_handler_);

void ActuatorModule::sendGotoAbsolutePositionPacket(uint16_t position, float velocity_feedforward,
                                                    uint16_t sub_device_id) {
    // Set the Actuator to position mode if needed to complete request
    if (_controlModes[sub_device_id] != ActuatorConstants::LENGTH_MODE) {
        ROIPackets::Packet packet = ROIPackets::Packet();
        packet.setClientAddressOctet(this->getOctet());
        packet.setSubDeviceID(sub_device_id);
        packet.setActionCode(ActuatorConstants::SET_CONTROL);
        packet.setData(ActuatorConstants::LENGTH_MODE);

        this->sendGeneralPacket(packet);

        _controlModes[sub_device_id] = ActuatorConstants::LENGTH_MODE;
    }

    if (_inputVelocities[sub_device_id] != velocity_feedforward) {
        // Set the velocity feedforward if it is different from the current one
        ROIPackets::Packet packet = ROIPackets::Packet();
        packet.setClientAddressOctet(this->getOctet());
        packet.setSubDeviceID(sub_device_id);
        packet.setActionCode(ActuatorConstants::SET_VELOCITY);
        packet.setData_impFloatCast(velocity_feedforward);

        this->sendGeneralPacket(packet);
        _inputVelocities[sub_device_id] = velocity_feedforward;
    }

    // Send the position set point
    ROIPackets::Packet packet = ROIPackets::Packet();
    packet.setClientAddressOctet(this->getOctet());
    packet.setSubDeviceID(sub_device_id);
    packet.setActionCode(ActuatorConstants::SET_ABSOLUTE_LENGTH);
    packet.setData_impSplit(position);
    packet.setSubDeviceID(sub_device_id);

    this->sendGeneralPacket(packet);
    _inputPositions[sub_device_id] = position;
}

void ActuatorModule::sendGotoRelativePositionPacket(uint16_t position, float velocity_feedforward,
                                                    uint16_t sub_device_id) {
    // Set the Actuator to position mode if needed to complete request
    if (_controlModes[sub_device_id] != ActuatorConstants::LENGTH_MODE) {
        ROIPackets::Packet packet = ROIPackets::Packet();
        packet.setClientAddressOctet(this->getOctet());
        packet.setSubDeviceID(sub_device_id);
        packet.setActionCode(ActuatorConstants::SET_CONTROL);
        packet.setData(ActuatorConstants::LENGTH_MODE);

        this->sendGeneralPacket(packet);

        _controlModes[sub_device_id] = ActuatorConstants::LENGTH_MODE;
    }

    if (_inputVelocities[sub_device_id] != velocity_feedforward) {
        // Set the velocity feedforward if it is different from the current one
        ROIPackets::Packet packet = ROIPackets::Packet();
        packet.setClientAddressOctet(this->getOctet());
        packet.setSubDeviceID(sub_device_id);
        packet.setActionCode(ActuatorConstants::SET_VELOCITY);
        packet.setData_impFloatCast(velocity_feedforward);

        this->sendGeneralPacket(packet);
        _inputVelocities[sub_device_id] = velocity_feedforward;
    }

    // Send the position set point
    ROIPackets::Packet packet = ROIPackets::Packet();
    packet.setClientAddressOctet(this->getOctet());

    packet.setActionCode(ActuatorConstants::SET_RELATIVE_LENGTH);
    packet.setData_impSplit(position);
    packet.setSubDeviceID(sub_device_id);

    this->sendGeneralPacket(packet);
    _inputPositions[sub_device_id] += position;  // Update the input position
    _relativeStartPositions[sub_device_id] =
        _positions[sub_device_id];  // Update the relative start position
}

void ActuatorModule::sendSetVelocityPacket(float velocity, uint16_t sub_device_id) {
    if (_controlModes[sub_device_id] != ActuatorConstants::VELOCITY_MODE) {
        ROIPackets::Packet packet = ROIPackets::Packet();
        packet.setClientAddressOctet(this->getOctet());
        packet.setActionCode(ActuatorConstants::SET_CONTROL);
        packet.setData(ActuatorConstants::VELOCITY_MODE);

        this->sendGeneralPacket(packet);
        _controlModes[sub_device_id] = ActuatorConstants::VELOCITY_MODE;
    }

    ROIPackets::Packet packet = ROIPackets::Packet();
    packet.setClientAddressOctet(this->getOctet());
    packet.setActionCode(ActuatorConstants::SET_VELOCITY);
    packet.setData_impFloatCast(velocity);
    packet.setSubDeviceID(sub_device_id);

    this->sendGeneralPacket(packet);
    _inputVelocities[sub_device_id] = velocity;  // Update the input velocity
}

void ActuatorModule::initializeTopics() {
    // Initialize the Actuator module
    this->debugLog("Initializing Actuator Module Topics");

    this->_controlModes.clear();
    this->_inputPositions.clear();
    this->_relativeStartPositions.clear();
    this->_inputVelocities.clear();
    this->_positions.clear();
    this->_velocities.clear();
    this->_homeElapsedTimes.clear();
    // Resize the vectors to the number of actuators
    uint8_t actuatorCount =
        this->get_parameter("actuator_count").get_parameter_value().get<uint8_t>();
    this->_controlModes.resize(actuatorCount, ActuatorConstants::LENGTH_MODE);
    this->_inputPositions.resize(actuatorCount, 0);
    this->_relativeStartPositions.resize(actuatorCount, 0);
    this->_inputVelocities.resize(actuatorCount, 0);
    this->_positions.resize(actuatorCount, 0);
    this->_velocities.resize(actuatorCount, 0);
    this->_homeElapsedTimes.resize(actuatorCount, 0);

    // Create the state publishers
    this->_state_publishers_.clear();
    this->_publishStateMessages.clear();

    // Create the duration publishers
    this->_home_elapsed_time_publishers_.clear();
    this->_publishHomeElapsedTimeMessages.clear();

    for (uint8_t i = 0;
         i < this->get_parameter("actuator_count").get_parameter_value().get<uint8_t>(); i++) {
        this->_state_publishers_.push_back(this->create_publisher<sensor_msgs::msg::JointState>(
            "roi_ros/act/axis" + std::to_string(i) + "/state", 10));

        // MSG ---------------
        //  State Publishers
        // Create lambda state publisher functions
        this->_publishStateMessages.push_back([=]() {  //[=] captures the "i" for each func. "this"
                                                       // is implicitly passed as a pointer
            // Publish the state message, position and velocity
            sensor_msgs::msg::JointState message = sensor_msgs::msg::JointState();
            message.name.push_back("axis" + std::to_string(i));
            message.position[0] = _positions[i] / 1000.0;   // Convert to meters
            message.velocity[0] = _velocities[i] / 1000.0;  // Convert to m/s
            this->_state_publishers_[i]->publish(message);
        });  // Add a lambda function to the vector to publish the state message

        // Create the home elapsed time publishers
        this->_home_elapsed_time_publishers_.push_back(
            this->create_publisher<builtin_interfaces::msg::Duration>(
                "roi_ros/act/axis" + std::to_string(i) + "/home_elapsed_time", 10));

        // Create lambda home elapsed time publisher functions
        this->_publishHomeElapsedTimeMessages.push_back([=]() {
            // Publish the home elapsed time message
            builtin_interfaces::msg::Duration message = builtin_interfaces::msg::Duration();
            message.sec = _homeElapsedTimes[i] / 1000;                  // Convert to seconds
            message.nanosec = (_homeElapsedTimes[i] % 1000) * 1000000;  // Convert to nanoseconds
            this->_home_elapsed_time_publishers_[i]->publish(message);
        });  // Add a lambda function to the vector to publish the home elapsed time message
    }
}

bool ActuatorModule::validateInput(float position, float velocity, uint16_t sub_device_id) {
    if (position < this->get_parameter("min_position")
                       .get_parameter_value()
                       .get<std::vector<float>>()[sub_device_id] ||
        position > this->get_parameter("max_position")
                       .get_parameter_value()
                       .get<std::vector<float>>()[sub_device_id]) {
        this->debugLog("Position out of bounds: " + std::to_string(position));
        return false;
    }
    if (abs(velocity) > this->get_parameter("max_velocity")
                            .get_parameter_value()
                            .get<std::vector<float>>()[sub_device_id]) {
        this->debugLog("Velocity out of bounds: " + std::to_string(velocity));
        return false;
    }

    return true;
}

void ActuatorModule::actuatorParameterCheck() {
    // Check the actuator parameters for changes

    // Check the actuator count parameter
    uint8_t actuatorCount =
        this->get_parameter("actuator_count").get_parameter_value().get<uint8_t>();
    if (actuatorCount != this->_controlModes.size()) {
        this->debugLog("Actuator count changed, reinitializing topics");
        this->initializeTopics();

        // match min and max position and velocity to the new actuator count
        auto minPosition =
            this->get_parameter("min_position").get_parameter_value().get<std::vector<float>>();
        auto maxPosition =
            this->get_parameter("max_position").get_parameter_value().get<std::vector<float>>();
        auto maxVelocity =
            this->get_parameter("max_velocity").get_parameter_value().get<std::vector<float>>();
        if (minPosition.size() != actuatorCount) {
            std::vector<float> newMinPosition(actuatorCount, 0.0);
            __matchPositionCopy(minPosition, newMinPosition);
            this->set_parameter(rclcpp::Parameter("min_position", newMinPosition));
        }
        if (maxPosition.size() != actuatorCount) {
            std::vector<float> newMaxPosition(actuatorCount, 1.0);
            __matchPositionCopy(maxPosition, newMaxPosition);
            this->set_parameter(rclcpp::Parameter("max_position", newMaxPosition));
        }
        if (maxVelocity.size() != actuatorCount) {
            std::vector<float> newMaxVelocity(actuatorCount, 0.1);
            __matchPositionCopy(maxVelocity, newMaxVelocity);
            this->set_parameter(rclcpp::Parameter("max_velocity", newMaxVelocity));
        }

        // match the velocity pid and position pid parameters to the new actuator count
        auto velocityPid =
            this->get_parameter("velocity_pid").get_parameter_value().get<std::vector<double>>();
        auto positionPid =
            this->get_parameter("position_pid").get_parameter_value().get<std::vector<double>>();

        if (velocityPid.size() != actuatorCount * 3) {
            std::vector<double> newVelocityPid(actuatorCount * 3, 0.1);
            __matchPositionCopy(velocityPid, newVelocityPid);
            this->set_parameter(rclcpp::Parameter("velocity_pid", newVelocityPid));
        }
        if (positionPid.size() != actuatorCount * 3) {
            std::vector<double> newPositionPid(actuatorCount * 3, 0.1);
            __matchPositionCopy(positionPid, newPositionPid);
            this->set_parameter(rclcpp::Parameter("position_pid", newPositionPid));
        }

        // Resize the pid memory vectors
        this->_positionPIDMemory.resize(actuatorCount);
        this->_velocityPIDMemory.resize(actuatorCount);
    }

    // Check the velocity pid parameter
    std::vector<double> velocityPidDouble =
        this->get_parameter("velocity_pid").get_parameter_value().get<std::vector<double>>();
    std::vector<float> velocityPid(velocityPidDouble.begin(), velocityPidDouble.end());
    if (!(velocityPid == _velocityPIDMemory)) {
        this->debugLog("Velocity PID parameters changed, pushing to actuators");
        ROIPackets::Packet packet = ROIPackets::Packet();
        packet.setClientAddressOctet(this->getOctet());
        packet.setActionCode(ActuatorConstants::SET_SPEED_PID);
        for (uint8_t i = 0; i < actuatorCount; i++) {
            packet.setSubDeviceID(i);
            uint8_t data[12];
            floatCast::floatToUint8Array(velocityPid[i * 3], data, 0, 3);      // p
            floatCast::floatToUint8Array(velocityPid[i * 3 + 1], data, 4, 7);  // i
            floatCast::floatToUint8Array(velocityPid[i * 3 + 2], data, 8, 11);
            packet.setData(data, 12);  // Set the data to the packet
            this->sendGeneralPacket(packet);
        }

        __matchPositionCopy(velocityPid, _velocityPIDMemory);
    }

    // Check the position pid parameter
    std::vector<double> positionPidDouble =
        this->get_parameter("position_pid").get_parameter_value().get<std::vector<double>>();
    std::vector<float> positionPid(positionPidDouble.begin(), positionPidDouble.end());
    if (!(positionPid == _positionPIDMemory)) {
        this->debugLog("Position PID parameters changed, pushing to actuators");
        ROIPackets::Packet packet = ROIPackets::Packet();
        packet.setClientAddressOctet(this->getOctet());
        packet.setActionCode(ActuatorConstants::SET_LENGTH_PID);
        for (uint8_t i = 0; i < actuatorCount; i++) {
            packet.setSubDeviceID(i);
            uint8_t data[12];
            floatCast::floatToUint8Array(positionPid[i * 3], data, 0, 3);      // p
            floatCast::floatToUint8Array(positionPid[i * 3 + 1], data, 4, 7);  // i
            floatCast::floatToUint8Array(positionPid[i * 3 + 2], data, 8, 11);
            packet.setData(data, 12);  // Set the data to the packet
            this->sendGeneralPacket(packet);
        }
        __matchPositionCopy(positionPid, _positionPIDMemory);
    }
}

//-------- PUBLIC METHODS --------//

ActuatorModule::ActuatorModule() : BaseModule("ActuatorModule", moduleTypesConstants::ACTUATOR) {
    // Initialize the Actuator module
    // this->debugLog("Initializing Actuator Module");

    this->declare_parameter("actuator_count", 1);
    this->declare_parameter("min_position", std::vector<float>{0});
    this->declare_parameter("max_position", std::vector<float>{1.0});
    this->declare_parameter("max_velocity", std::vector<float>{0.1});
    this->declare_parameter("velocity_pid", std::vector<float>{0.1, 0.01, 0.001});
    this->declare_parameter("position_pid", std::vector<float>{0.1, 0.01, 0.001});

    this->initializeTopics();  // Initialize the dynamic ros interfaces.

    _parameterTimer =
        this->create_wall_timer(  // Create a timer to check the parameters for changes
            std::chrono::milliseconds(WatchdogConstants::MAINTAIN_SLEEP_TIME * 15),
            std::bind(&ActuatorModule::actuatorParameterCheck, this));

    // Send a status report to check module state
    ROIPackets::sysAdminPacket statusPacket = ROIPackets::sysAdminPacket();
    statusPacket.setAdminMetaData(sysAdminConstants::NO_CHAIN_META);
    statusPacket.setActionCode(sysAdminConstants::STATUS_REPORT);
    statusPacket.setClientAddressOctet(this->getOctet());

    this->sendSysadminPacket(statusPacket);

    this->debugLog("Actuator Module Initialized");
}

ActuatorModule::~ActuatorModule() {
    // Destroy the GPIO module
    this->debugLog("Destroying Actuator Module");
}

bool ActuatorModule::pushState() {
    // Push the current stored state of the module to the physical module
    // This is used to recover the state of the module after a hardware reset but Ros node still
    // alive.

    this->debugLog("Pushing state to Actuator module");

    ROIPackets::Packet controlMode = ROIPackets::Packet();
    controlMode.setActionCode(ActuatorConstants::SET_CONTROL);
    controlMode.setClientAddressOctet(this->getOctet());

    ROIPackets::Packet absTargetPosition = ROIPackets::Packet();
    absTargetPosition.setActionCode(ActuatorConstants::SET_ABSOLUTE_LENGTH);
    absTargetPosition.setClientAddressOctet(this->getOctet());

    ROIPackets::Packet velocity = ROIPackets::Packet();
    velocity.setActionCode(ActuatorConstants::SET_VELOCITY);
    velocity.setClientAddressOctet(this->getOctet());

    ROIPackets::Packet setSpeedPID = ROIPackets::Packet();
    setSpeedPID.setActionCode(ActuatorConstants::SET_SPEED_PID);
    setSpeedPID.setClientAddressOctet(this->getOctet());

    ROIPackets::Packet setLengthPID = ROIPackets::Packet();
    setLengthPID.setActionCode(ActuatorConstants::SET_LENGTH_PID);
    setLengthPID.setClientAddressOctet(this->getOctet());

    for (uint8_t i = 0; i < this->_controlModes.size(); i++) {
        // Set the control mode
        controlMode.setSubDeviceID(i);
        controlMode.setData(_controlModes[i]);
        this->sendGeneralPacket(controlMode);

        // Set the absolute target position
        absTargetPosition.setSubDeviceID(i);
        absTargetPosition.setData_impSplit(_inputPositions[i]);
        this->sendGeneralPacket(absTargetPosition);

        // Set the velocity
        velocity.setSubDeviceID(i);
        velocity.setData_impFloatCast(_inputVelocities[i]);
        this->sendGeneralPacket(velocity);

        // Set the speed PID
        setSpeedPID.setSubDeviceID(i);
        uint8_t data[12];
        floatCast::floatToUint8Array(_velocityPIDMemory[i * 3], data, 0, 3);       // p
        floatCast::floatToUint8Array(_velocityPIDMemory[i * 3 + 1], data, 4, 7);   // i
        floatCast::floatToUint8Array(_velocityPIDMemory[i * 3 + 2], data, 8, 11);  // d
        setSpeedPID.setData(data, 12);  // Set the data to the packet
        this->sendGeneralPacket(setSpeedPID);

        // Set the length PID
        setLengthPID.setSubDeviceID(i);
        floatCast::floatToUint8Array(_positionPIDMemory[i * 3], data, 0, 3);       // p
        floatCast::floatToUint8Array(_positionPIDMemory[i * 3 + 1], data, 4, 7);   // i
        floatCast::floatToUint8Array(_positionPIDMemory[i * 3 + 2], data, 8, 11);  // d
        setLengthPID.setData(data, 12);  // Set the data to the packet
        this->sendGeneralPacket(setLengthPID);
    }

    this->debugLog("State pushed to Actuator module");

    return true;
}

bool ActuatorModule::pullState() {
    // Pull the current state of the module from the physical module. Used to recover a ROS node
    // restart when the module is still running

    this->debugLog("Pulling state from Actuator module");

    ROIPackets::Packet control = ROIPackets::Packet();
    control.setClientAddressOctet(this->getOctet());
    control.setActionCode(ActuatorConstants::GET_CONTROL);

    ROIPackets::Packet absLength = ROIPackets::Packet();
    absLength.setClientAddressOctet(this->getOctet());
    absLength.setActionCode(ActuatorConstants::GET_TARGET_LENGTH);

    ROIPackets::Packet velocity = ROIPackets::Packet();
    velocity.setClientAddressOctet(this->getOctet());
    velocity.setActionCode(ActuatorConstants::GET_TARGET_VELOCITY);

    ROIPackets::Packet speedPID = ROIPackets::Packet();
    speedPID.setClientAddressOctet(this->getOctet());
    speedPID.setActionCode(ActuatorConstants::GET_SPEED_PID);

    ROIPackets::Packet lengthPID = ROIPackets::Packet();
    lengthPID.setClientAddressOctet(this->getOctet());
    lengthPID.setActionCode(ActuatorConstants::GET_LENGTH_PID);

    for (uint8_t i = 0; i < this->_controlModes.size(); i++) {
        // Get the control mode
        control.setSubDeviceID(i);
        this->sendGeneralPacket(control);

        // Get the absolute target position
        absLength.setSubDeviceID(i);
        this->sendGeneralPacket(absLength);

        // Get the velocity
        velocity.setSubDeviceID(i);
        this->sendGeneralPacket(velocity);

        // Get the speed PID
        speedPID.setSubDeviceID(i);
        this->sendGeneralPacket(speedPID);

        // Get the length PID
        lengthPID.setSubDeviceID(i);
        this->sendGeneralPacket(lengthPID);
    }

    this->debugLog("State pulled from Actuator module");

    return true;
}

#ifdef ACTUATOR_MODULE
// We may wish to include and extend upon the actuator module in the future, in which case we do not
// wish to execute this main. Disable by not defining ACTUATOR_MODULE in CMAKELists.txt
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActuatorModule>());
    rclcpp::shutdown();
    return 0;
}
#endif  // ACTUATOR_MODULE