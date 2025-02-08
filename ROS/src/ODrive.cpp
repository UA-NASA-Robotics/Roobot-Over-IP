#include "ODrive.h"

//-------- PRIVATE METHODS --------//

rcl_interfaces::msg::SetParametersResult ODriveModule::octetParameterCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
    // Handle the octet parameter change
    this->debugLog("Octet parameter changed to " + std::to_string(this->getOctet()));

    // Unsubscribe from the old response topic
    this->_response_subscription_.reset();  // release pointer and gc the old subscription
    // Subscribe to the new response topic, at the new octet
    this->_response_subscription_ = this->create_subscription<roi_ros::msg::SerializedPacket>(
        "octet" + std::to_string(this->getOctet()) + "_response", 10,
        std::bind(&ODriveModule::responseCallback, this, std::placeholders::_1));

    // Unsubscribe from the old sysadmin response topic
    this->_sysadmin_response_subscription_.reset();  // release pointer and gc the old subscription
    // Subscribe to the new sysadmin response topic, at the new octet
    this->_sysadmin_response_subscription_ =
        this->create_subscription<roi_ros::msg::SerializedPacket>(
            "sys_admin_octet" + std::to_string(this->getOctet()) + "_response", 10,
            std::bind(&ODriveModule::sysadminResponseCallback, this, std::placeholders::_1));

    // Unsubscribe from the old connection state topic
    this->_connection_state_subscription_.reset();  // release pointer and gc the old subscription
    // Subscribe to the new connection state topic, at the new octet
    this->_connection_state_subscription_ =
        this->create_subscription<roi_ros::msg::ConnectionState>(
            "octet" + std::to_string(this->getOctet()) + "_connection_state", 10,
            std::bind(&BaseModule::connectionStateCallback, this, std::placeholders::_1));

    // synchronize with the new module
    this->pushState();

    this->debugLog("Octet parameter change handled");

    return rcl_interfaces::msg::SetParametersResult();
}

void ODriveModule::maintainState() {
    // Maintain the state of the ODrive module

    // Loop to maintain the state
    uint8_t checkResetCounter = 0;
    while (rclcpp::ok()) {
        // Check if the module has been reset, once every 128 loops (128*50ms = 6.4s)
        if (checkResetCounter == 0) {
            // Issues a status report request. The callback will handle the response.
            // If the callback receives a BLANKSTATE status, it will push the current state to the
            // module.
            ROIPackets::sysAdminPacket statusPacket = ROIPackets::sysAdminPacket();
            statusPacket.setAdminMetaData(sysAdminConstants::NOCHAINMETA);
            statusPacket.setActionCode(sysAdminConstants::STATUSREPORT);

            this->sendSysadminPacket(statusPacket);
            checkResetCounter = 128;
        }
        checkResetCounter--;  // Increment the check reset counter

        // Loop through all of the readable values and request their values
        ROIPackets::Packet readPacket = ROIPackets::Packet();
        readPacket.setActionCode(ODriveConstants::GETALL);
        this->sendGeneralPacket(readPacket);

        // Sleep for n seconds
        std::this_thread::sleep_for(
            std::chrono::milliseconds(WatchdogConstants::MAINTAIN_SLEEP_TIME));
    }
}

void ODriveModule::responseCallback(const roi_ros::msg::SerializedPacket response) {
    // Handle the response from the transport agent
    this->debugLog("Received response from transport agent");

    // Parse the response packet
    ROIPackets::Packet packet = ROIPackets::Packet();
    uint8_t serializedData[ROIConstants::ROIMAXPACKETSIZE];
    this->unpackVectorToArray(response.data, serializedData, response.length);
    if (!packet.importPacket(serializedData, response.length) &&
        !moduleNodeConstants::ignoreMalformedPackets) {
        this->debugLog("Failed to import packet");
        return;
    }

    // Handle the response packet
    uint8_t data[ROIConstants::ROIMAXPACKETPAYLOAD];
    packet.getData(data, ROIConstants::ROIMAXPACKETPAYLOAD);
    if (packet.getActionCode() & ODriveConstants::MaskConstants::GETMASK) {
        // Handle the response to a get request
        switch (packet.getActionCode() & !ODriveConstants::MaskConstants::GETMASK) {
            case ODriveConstants::MaskConstants::ControlMode:
                _controlMode = data[0];
                break;

            case ODriveConstants::MaskConstants::InputMode:
                _inputMode = data[0];
                break;

            case ODriveConstants::MaskConstants::Torque:
                _inputTorque = floatCast::toFloat(data, 0, 3);
                break;

            case ODriveConstants::MaskConstants::PositionSetPoint:
                _inputPosition = floatCast::toFloat(data, 0, 3);
                break;

            case ODriveConstants::MaskConstants::VelocitySetPoint:
                _inputVelocity = floatCast::toFloat(data, 0, 3);
                break;

            case ODriveConstants::MaskConstants::Error:
                _errorCode = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];

                this->publishHealthMessage();
                break;

            case ODriveConstants::MaskConstants::Velocity:
                _velocity = floatCast::toFloat(data, 0, 3);

                this->publishStateMessage();
                break;

            case ODriveConstants::MaskConstants::Position:
                _position = floatCast::toFloat(data, 0, 3);

                this->publishStateMessage();
                break;

            case ODriveConstants::MaskConstants::BusVoltage:
                _busVoltage = floatCast::toFloat(data, 0, 3);

                this->publishPowerMessage();
                break;

            case ODriveConstants::MaskConstants::Current:
                _current = floatCast::toFloat(data, 0, 3);

                this->publishPowerMessage();
                break;

            case ODriveConstants::MaskConstants::MotorTemperature:
                _motorTemperature = floatCast::toFloat(data, 0, 3);

                this->publishTemperatureMessage();
                break;

            case ODriveConstants::MaskConstants::FETTemperature:
                _fetTemperature = floatCast::toFloat(data, 0, 3);

                this->publishTemperatureMessage();
                break;

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

                this->publishHealthMessage();
                this->publishPowerMessage();
                this->publishTemperatureMessage();
                this->publishStateMessage();

                break;

            default:
                this->debugLog("Unknown get action code received: " +
                               std::to_string(packet.getActionCode()));
                break;
        }
    } else {
        // Handle the response to a set request
        switch (packet.getActionCode() & !ODriveConstants::MaskConstants::GETMASK) {
            case ODriveConstants::MaskConstants::ControlMode:
                this->debugLog("Control mode set");
                if (!data[0]) {
                    this->debugLog("Controlmode set failure.");
                    _module_error_message = "Controlmode set failure.";
                    this->publishHealthMessage();
                }
                break;

            case ODriveConstants::MaskConstants::InputMode:
                this->debugLog("Input mode set");
                if (!data[0]) {
                    this->debugLog("Input mode set failure.");
                    _module_error_message = "Input mode set failure.";
                    this->publishHealthMessage();
                }
                break;

            case ODriveConstants::MaskConstants::Torque:
                this->debugLog("Torque set");
                if (!data[0]) {
                    this->debugLog("Torque set failure.");
                    _module_error_message = "Torque set failure.";
                    this->publishHealthMessage();
                }
                break;

            case ODriveConstants::MaskConstants::PositionSetPoint:
                this->debugLog("Position set");
                if (!data[0]) {
                    this->debugLog("Position set failure.");
                    _module_error_message = "Position set failure.";
                    this->publishHealthMessage();
                }
                break;

            case ODriveConstants::MaskConstants::VelocitySetPoint:
                this->debugLog("Velocity set");
                if (!data[0]) {
                    this->debugLog("Velocity set failure.");
                    _module_error_message = "Velocity set failure.";
                    this->publishHealthMessage();
                }
                break;

            case ODriveConstants::MaskConstants::PositionRelative:
                this->debugLog("Relative position set");
                if (!data[0]) {
                    this->debugLog("Relative position set failure.");
                    _module_error_message = "Relative position set failure.";
                    this->publishHealthMessage();
                }
                break;

            case ODriveConstants::MaskConstants::Error:
                this->debugLog("Error cleared");
                if (!data[0]) {
                    this->debugLog("Error clear failure.");
                    _module_error_message = "Error clear failure.";
                    this->publishHealthMessage();
                }
                break;

            default:
                this->debugLog("Unknown action code received: " +
                               std::to_string(packet.getActionCode()));
                break;
        }
    }

    this->debugLog("Response handled");
}

void ODriveModule::publishPowerMessage() {
    // Publish the power message, voltage and current
    auto message = roi_ros::msg::Power();
    message.voltage = _busVoltage;
    message.current = _current;
    this->_power_publisher_->publish(message);
}

void ODriveModule::publishStateMessage() {
    // Publish the state message, position and velocity
    auto message = roi_ros::msg::State();
    message.position = _position;
    message.velocity = _velocity;
    this->_state_publisher_->publish(message);
}

void ODriveModule::publishTemperatureMessage() {
    // Publish the temperature message, motor and fet
    auto message = roi_ros::msg::Temperature();
    message.motor = _motorTemperature;
    message.fet = _fetTemperature;
    this->_temperature_publisher_->publish(message);
}

void ODriveModule::gotoPositionServiceHandler(
    const roi_ros::srv::ODriveGotoPosition::Request::SharedPtr request,
    roi_ros::srv::ODriveGotoPosition::Response::SharedPtr response) {
    // Handle the goto position service request
    this->debugLog("Received goto position service request");

    this->sendGotoPositionPacket(request->position, request->velocity_feedforward,
                                 request->torque_feedforward);

    // Respond to the service request
    response->success =
        !_module_error;  // if there is an error, success is false, we may have not done the request
    this->debugLog("Goto position service request handled");
}

void ODriveModule::gotoRelativePositionServiceHandler(
    const roi_ros::srv::ODriveGotoRelativePosition::Request::SharedPtr request,
    roi_ros::srv::ODriveGotoRelativePosition::Response::SharedPtr response) {
    // Handle the goto position service request
    this->debugLog("Received goto position service request");

    this->sendGotoRelativePositionPacket(request->position, request->velocity_feedforward,
                                         request->torque_feedforward);

    // Respond to the service request
    response->success =
        !_module_error;  // if there is an error, success is false, we may have not done the request
    this->debugLog("Goto position service request handled");
}

void ODriveModule::setTorqueServiceHandler(
    const roi_ros::srv::ODriveSetTorque::Request::SharedPtr request,
    roi_ros::srv::ODriveSetTorque::Response::SharedPtr response) {
    // Handle the set torque service request
    this->debugLog("Received set torque service request");

    this->sendSetTorquePacket(request->torque);

    // Respond to the service request
    response->success = !_module_error;

    this->debugLog("Set torque service request handled");
}

void ODriveModule::setVelocityServiceHandler(
    const roi_ros::srv::ODriveSetVelocity::Request::SharedPtr request,
    roi_ros::srv::ODriveSetVelocity::Response::SharedPtr response) {
    // Handle the set velocity service request
    this->debugLog("Received set velocity service request");

    this->sendSetVelocityPacket(request->velocity, request->torque_feedforward);

    // Respond to the service request
    response->success = !_module_error;

    this->debugLog("Set velocity service request handled");
}

void ODriveModule::sendGotoPositionPacket(float position, float velocity_feedforward,
                                          float torque_feedforward) {
    // Set the odrive to position mode if needed to complete request
    if (_controlMode != ODriveConstants::POSITIONMODE) {
        ROIPackets::Packet packet = ROIPackets::Packet();
        packet.setClientAddressOctet(this->getOctet());
        packet.setActionCode(ODriveConstants::SETCONTROLMODE);
        packet.setData(ODriveConstants::POSITIONMODE);

        this->sendGeneralPacket(packet);

        _controlMode = ODriveConstants::POSITIONMODE;
    }
    if (_inputMode != ODriveConstants::TRAP_TRAJ_MODE ||
        _inputMode != ODriveConstants::POS_FILTER_MODE) {
        ROIPackets::Packet packet = ROIPackets::Packet();
        packet.setClientAddressOctet(this->getOctet());
        packet.setActionCode(ODriveConstants::SETINPUTMODE);
        packet.setData(ODriveConstants::TRAP_TRAJ_MODE);

        this->sendGeneralPacket(packet);

        _inputMode = ODriveConstants::TRAP_TRAJ_MODE;
    }

    if (torque_feedforward != 0) {  // we have a torque feedforward to contribute
        ROIPackets::Packet packet = ROIPackets::Packet();
        packet.setClientAddressOctet(this->getOctet());
        packet.setActionCode(ODriveConstants::SETTORQUE);
        packet.setData(torque_feedforward);

        this->sendGeneralPacket(packet);
    }
    if (velocity_feedforward != 0) {  // we have a velocity feedforward to contribute
        ROIPackets::Packet packet = ROIPackets::Packet();
        packet.setClientAddressOctet(this->getOctet());
        packet.setActionCode(ODriveConstants::SETVELOCITY);
        packet.setData(velocity_feedforward);

        this->sendGeneralPacket(packet);
    }

    // Send the position set point
    ROIPackets::Packet packet = ROIPackets::Packet();
    packet.setClientAddressOctet(this->getOctet());
    packet.setActionCode(ODriveConstants::SETPOSITION);
    packet.setData(position);

    this->sendGeneralPacket(packet);
}

void ODriveModule::sendGotoRelativePositionPacket(float position, float velocity_feedforward,
                                                  float torque_feedforward) {
    // Set the odrive to position mode if needed to complete request
    if (_controlMode != ODriveConstants::POSITIONMODE) {
        ROIPackets::Packet packet = ROIPackets::Packet();
        packet.setClientAddressOctet(this->getOctet());
        packet.setActionCode(ODriveConstants::SETCONTROLMODE);
        packet.setData(ODriveConstants::POSITIONMODE);

        this->sendGeneralPacket(packet);

        _controlMode = ODriveConstants::POSITIONMODE;
    }
    if (_inputMode != ODriveConstants::TRAP_TRAJ_MODE ||
        _inputMode != ODriveConstants::POS_FILTER_MODE) {
        ROIPackets::Packet packet = ROIPackets::Packet();
        packet.setClientAddressOctet(this->getOctet());
        packet.setActionCode(ODriveConstants::SETINPUTMODE);
        packet.setData(ODriveConstants::TRAP_TRAJ_MODE);

        this->sendGeneralPacket(packet);

        _inputMode = ODriveConstants::TRAP_TRAJ_MODE;
    }

    if (torque_feedforward != 0) {  // we have a torque feedforward to contribute
        ROIPackets::Packet packet = ROIPackets::Packet();
        packet.setClientAddressOctet(this->getOctet());
        packet.setActionCode(ODriveConstants::SETTORQUE);
        packet.setData(torque_feedforward);

        this->sendGeneralPacket(packet);
    }
    if (velocity_feedforward != 0) {  // we have a velocity feedforward to contribute
        ROIPackets::Packet packet = ROIPackets::Packet();
        packet.setClientAddressOctet(this->getOctet());
        packet.setActionCode(ODriveConstants::SETVELOCITY);
        packet.setData(velocity_feedforward);

        this->sendGeneralPacket(packet);
    }

    // Send the position set point
    ROIPackets::Packet packet = ROIPackets::Packet();
    packet.setClientAddressOctet(this->getOctet());
    packet.setActionCode(ODriveConstants::SETRELATIVEPOSITION);
    packet.setData(position);

    this->sendGeneralPacket(packet);
}

void ODriveModule::sendSetTorquePacket(float torque) {
    if (_controlMode != ODriveConstants::TORQUEMODE) {
        ROIPackets::Packet packet = ROIPackets::Packet();
        packet.setClientAddressOctet(this->getOctet());
        packet.setActionCode(ODriveConstants::SETCONTROLMODE);
        packet.setData(ODriveConstants::TORQUEMODE);

        this->sendGeneralPacket(packet);
        _controlMode = ODriveConstants::TORQUEMODE;
    }

    if (_inputMode != ODriveConstants::TORQUE_RAMP_MODE) {
        ROIPackets::Packet packet = ROIPackets::Packet();
        packet.setClientAddressOctet(this->getOctet());
        packet.setActionCode(ODriveConstants::SETINPUTMODE);
        packet.setData(ODriveConstants::TORQUE_RAMP_MODE);

        this->sendGeneralPacket(packet);
        _inputMode = ODriveConstants::TORQUE_RAMP_MODE;
    }

    ROIPackets::Packet packet = ROIPackets::Packet();
    packet.setClientAddressOctet(this->getOctet());
    packet.setActionCode(ODriveConstants::SETTORQUE);
    packet.setData(torque);

    this->sendGeneralPacket(packet);
}

void ODriveModule::sendSetVelocityPacket(float velocity, float torque_feedforward) {
    if (_controlMode != ODriveConstants::VELOCITYMODE) {
        ROIPackets::Packet packet = ROIPackets::Packet();
        packet.setClientAddressOctet(this->getOctet());
        packet.setActionCode(ODriveConstants::SETCONTROLMODE);
        packet.setData(ODriveConstants::VELOCITYMODE);

        this->sendGeneralPacket(packet);
        _controlMode = ODriveConstants::VELOCITYMODE;
    }

    if (_inputMode != ODriveConstants::VELOCITY_RAMP_MODE) {
        ROIPackets::Packet packet = ROIPackets::Packet();
        packet.setClientAddressOctet(this->getOctet());
        packet.setActionCode(ODriveConstants::SETINPUTMODE);
        packet.setData(ODriveConstants::VELOCITY_RAMP_MODE);

        this->sendGeneralPacket(packet);
        _inputMode = ODriveConstants::VELOCITY_RAMP_MODE;
    }

    if (torque_feedforward != 0) {
        ROIPackets::Packet packet = ROIPackets::Packet();
        packet.setClientAddressOctet(this->getOctet());
        packet.setActionCode(ODriveConstants::SETTORQUE);
        packet.setData(torque_feedforward);

        this->sendGeneralPacket(packet);
    }

    ROIPackets::Packet packet = ROIPackets::Packet();
    packet.setClientAddressOctet(this->getOctet());
    packet.setActionCode(ODriveConstants::SETVELOCITY);
    packet.setData(velocity);

    this->sendGeneralPacket(packet);
}

rclcpp_action::GoalResponse ODriveModule::gotoPositionGoalHandler(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const roi_ros::action::ODriveGotoPosition::Goal> goal) {
    this->debugLog("Received goto position action goal request");

    (void)uuid;  // unused??? tf does this do

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;  // accept and execute any goal. we're
                                                             // not picky
}

void ODriveModule::gotoPositionAcceptedHandler(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<roi_ros::action::ODriveGotoPosition>>
        goalHandle) {
    this->debugLog(
        "Received goto position action accepted request. Spinning up execution monitor thread");

    // initiate the movement asap. don't wait for the thread to start
    this->sendGotoPositionPacket(
        goalHandle->get_goal()->position, goalHandle->get_goal()->velocity_feedforward,
        goalHandle->get_goal()->torque_feedforward);  // send the goal to the module

    std::thread(&ODriveModule::gotoPositionExecuteHandler, this, goalHandle)
        .detach();  // spin up the execution thread
}

rclcpp_action::CancelResponse ODriveModule::gotoPositionCancelHandler(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<roi_ros::action::ODriveGotoPosition>>
        goalHandle) {
    this->debugLog("Received goto position action cancel request");

    this->sendSetVelocityPacket(0, 0);  // stop the motor

    (void)goalHandle;  // ig remove the goal handle

    return rclcpp_action::CancelResponse::ACCEPT;  // accept any cancel request
}

void ODriveModule::gotoPositionExecuteHandler(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<roi_ros::action::ODriveGotoPosition>>
        goalHandle) {
    this->debugLog("Executing goto position action goal loop");
    const auto goal = goalHandle->get_goal();  // get the goal
    auto feedback =
        std::make_shared<roi_ros::action::ODriveGotoPosition::Feedback>();  // create a feedback
                                                                            // object
    auto result =
        std::make_shared<roi_ros::action::ODriveGotoPosition::Result>();  // create a result object

    // loop until the goal is complete
    while (rclcpp::ok() &&
           (this->_velocity > 0.5 ||
            this->_position == goal.position)) {  // check if the node is still running and the
                                                  // velocity is above 0.5 rev/s
        // update the feedback
        feedback->current_position = this->_position;
        feedback->current_velocity = this->_velocity;
        goalHandle->publish_feedback(feedback);  // publish the feedback

        // sleep for 75ms
        std::this_thread::sleep_for(
            std::chrono::milliseconds(75));  // sleep for 75ms we don't need to be that fast. This
                                             // ensures the feedback is read between publishes

        // check if cancel has been requested
        if (goalHandle->is_canceling() ||
            this->_module_error) {  // check if the goal has been canceled or there is an error. If
                                    // so, cancel the goal
            result->success = false;       // set the result to false
            goalHandle->canceled(result);  // cancel the goal (this calls the cancel callback)
            this->debugLog("Goto position action goal canceled");
            return;  // exit the loop
        }
    }

    // if the loop exits, the goal is complete
    result->success = true;       // set the result to true
    goalHandle->succeed(result);  // succeed the goal
    this->debugLog("Goto position action goal succeeded");
}

rclcpp_action::GoalResponse ODriveModule::gotoRelativePositionGoalHandler(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const roi_ros::action::ODriveGotoRelativePosition::Goal> goal) {
    this->debugLog("Received goto relative position action goal request");

    (void)uuid;  // unused??? tf does this do

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;  // accept and execute any goal. we're
                                                             // not picky
}

void ODriveModule::gotoRelativePositionAcceptedHandler(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<roi_ros::action::ODriveGotoRelativePosition>>
        goalHandle) {
    this->debugLog(
        "Received goto relative position action accepted request. Spinning up execution monitor "
        "thread");

    this->_relativeStartPosition =
        this->_position;  // store the current position as the start position

    // initiate the movement asap. don't wait for the thread to start
    this->sendGotoRelativePositionPacket(
        goalHandle->get_goal()->position, goalHandle->get_goal()->velocity_feedforward,
        goalHandle->get_goal()->torque_feedforward);  // send the goal to the module

    std::thread(&ODriveModule::gotoRelativePositionExecuteHandler, this, goalHandle)
        .detach();  // spin up the execution thread
}

rclcpp_action::CancelResponse ODriveModule::gotoRelativePositionCancelHandler(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<roi_ros::action::ODriveGotoRelativePosition>>
        goalHandle) {
    this->debugLog("Received goto relative position action cancel request");

    this->sendSetVelocityPacket(0, 0);  // stop the motor

    (void)goalHandle;  // ig remove the goal handle

    return rclcpp_action::CancelResponse::ACCEPT;  // accept any cancel request
}

void ODriveModule::gotoRelativePositionExecuteHandler(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<roi_ros::action::ODriveGotoRelativePosition>>
        goalHandle) {
    this->debugLog("Executing goto relative position action goal loop");
    const auto goal = goalHandle->get_goal();  // get the goal
    auto feedback =
        std::make_shared<roi_ros::action::ODriveGotoRelativePosition::Feedback>();  // create a
                                                                                    // feedback
                                                                                    // object
    auto result =
        std::make_shared<roi_ros::action::ODriveGotoRelativePosition::Result>();  // create a result
                                                                                  // object

    // loop until the goal is complete
    while (rclcpp::ok() &&
           (this->_velocity > 0.5 ||
            abs(goal->position - this->_position -
                this->_relativeStartPosition))) {  // check if the node is still running and the
        // velocity is above 0.5 rev/s or the position is not reached
        // update the feedback
        feedback->current_position =
            this->_position - this->_relativeStartPosition;  // relative position
        feedback->current_velocity = this->_velocity;
        goalHandle->publish_feedback(feedback);  // publish the feedback

        // sleep for 75ms
        std::this_thread::sleep_for(
            std::chrono::milliseconds(75));  // sleep for 75ms we don't need to be that fast. This
                                             // ensures the feedback is read between publishes

        // check if cancel has been requested
        if (goalHandle->is_canceling() ||
            this->_module_error) {  // check if the goal has been canceled or there is an error. If
                                    // so, cancel the goal
            result->success = false;       // set the result to false
            goalHandle->canceled(result);  // cancel the goal (this calls the cancel callback)
            this->debugLog("Goto relative position action goal canceled");
            return;  // exit the loop
        }
    }

    // if the loop exits, the goal is complete
    result->success = true;       // set the result to true
    goalHandle->succeed(result);  // succeed the goal
    this->debugLog("Goto relative position action goal succeeded");
}

//-------- PUBLIC METHODS --------//

ODriveModule::ODriveModule() : BaseModule("ODriveModule") {
    // Initialize the ODrive module
    this->debugLog("Initializing ODrive Module");

    // Initialize the network parameters and callbacks
    this->declare_parameter("module_octet", 5);
    this->_octetParameterCallbackHandle = this->add_on_set_parameters_callback(
        std::bind(&ODriveModule::octetParameterCallback, this, std::placeholders::_1));

    // Initialize the module health publisher
    this->_health_publisher_ = this->create_publisher<roi_ros::msg::Health>("health", 10);

    // Initialize the module general packet queue client
    this->_queue_general_packet_client_ =
        this->create_client<roi_ros::srv::QueueSerializedGeneralPacket>("queue_general_packet");

    this->_queue_sysadmin_packet_client_ =
        this->create_client<roi_ros::srv::QueueSerializedSysAdminPacket>("queue_sys_admin_packet");

    // Initialize the base module response subscriptions (gets data from the transport agent)
    this->_response_subscription_ = this->create_subscription<roi_ros::msg::SerializedPacket>(
        "octet5_response", 10,
        std::bind(&ODriveModule::responseCallback, this, std::placeholders::_1));
    this->_sysadmin_response_subscription_ =
        this->create_subscription<roi_ros::msg::SerializedPacket>(
            "sys_admin_octet5_response", 10,
            std::bind(&ODriveModule::sysadminResponseCallback, this, std::placeholders::_1));

    this->_connection_state_subscription_ =
        this->create_subscription<roi_ros::msg::ConnectionState>(
            "octet5_connection_state", 10,
            std::bind(&BaseModule::connectionStateCallback, this, std::placeholders::_1));

    // Initialize the ODrive specific ros topics

    // Initialize the ODrive specific publishers
    this->_power_publisher_ = this->create_publisher<roi_ros::msg::Power>("power", 10);
    this->_state_publisher_ = this->create_publisher<roi_ros::msg::State>("state", 10);
    this->_temperature_publisher_ =
        this->create_publisher<roi_ros::msg::Temperature>("temperature", 10);

    // Initialize the ODrive specific services
    this->_goto_position_service_ = this->create_service<roi_ros::srv::ODriveGotoPosition>(
        "goto_position", std::bind(&ODriveModule::gotoPositionServiceHandler, this,
                                   std::placeholders::_1, std::placeholders::_2));
    this->_goto_relative_position_service_ =
        this->create_service<roi_ros::srv::ODriveGotoRelativePosition>(
            "goto_relative_position",
            std::bind(&ODriveModule::gotoRelativePositionServiceHandler, this,
                      std::placeholders::_1, std::placeholders::_2));
    this->_set_torque_service_ = this->create_service<roi_ros::srv::ODriveSetTorque>(
        "set_torque", std::bind(&ODriveModule::setTorqueServiceHandler, this, std::placeholders::_1,
                                std::placeholders::_2));
    this->_set_velocity_service_ = this->create_service<roi_ros::srv::ODriveSetVelocity>(
        "set_velocity", std::bind(&ODriveModule::setVelocityServiceHandler, this,
                                  std::placeholders::_1, std::placeholders::_2));

    // Initialize the ODrive specific action servers
    this->_goto_position_action_server_ =
        rclcpp_action::create_server<roi_ros::action::ODriveGotoPosition>(
            this, "goto_position",
            std::bind(&ODriveModule::gotoPositionGoalHandler, this, std::placeholders::_1,
                      std::placeholders::_2),
            std::bind(&ODriveModule::gotoPositionAcceptedHandler, this, std::placeholders::_1),
            std::bind(&ODriveModule::gotoPositionCancelHandler, this, std::placeholders::_1));
    this->_goto_relative_position_action_server_ =
        rclcpp_action::create_server<roi_ros::action::ODriveGotoRelativePosition>(
            this, "goto_relative_position",
            std::bind(&ODriveModule::gotoRelativePositionGoalHandler, this, std::placeholders::_1,
                      std::placeholders::_2),
            std::bind(&ODriveModule::gotoRelativePositionAcceptedHandler, this,
                      std::placeholders::_1),
            std::bind(&ODriveModule::gotoRelativePositionCancelHandler, this,
                      std::placeholders::_1));

    this->debugLog("ODrive Module Initialized");

    // Initialize the GPIO module maintain state thread
    this->_maintainStateThread =
        std::thread(&ODriveModule::maintainState, this);  // Spin up the maintain state thread
}

ODriveModule::~ODriveModule() {
    // Destroy the GPIO module
    this->debugLog("Destroying General GPIO Module");
    this->_maintainStateThread
        .join();  // Wait for the maintain state thread to finish (It should stop itself)
}

bool ODriveModule::pushState() {
    // Push the current stored state of the module to the physical module
    // This is used to recover the state of the module after a hardware reset but Ros node still
    // alive.

    this->debugLog("Pushing state to ODrive module");

    // Push the control mode
    ROIPackets::Packet packet = ROIPackets::Packet();
    packet.setClientAddressOctet(this->getOctet());
    packet.setActionCode(ODriveConstants::SETCONTROLMODE);
    packet.setData(_controlMode);
    this->sendGeneralPacket(packet);

    // Push the input mode
    packet.setActionCode(ODriveConstants::SETINPUTMODE);
    packet.setData(_inputMode);
    this->sendGeneralPacket(packet);

    // Push the input torque
    packet.setActionCode(ODriveConstants::SETTORQUE);
    packet.setData(_inputTorque);
    this->sendGeneralPacket(packet);

    // Push the input position
    packet.setActionCode(ODriveConstants::SETPOSITION);
    packet.setData(_inputPosition);
    this->sendGeneralPacket(packet);

    // Push the input velocity
    packet.setActionCode(ODriveConstants::SETVELOCITY);
    packet.setData(_inputVelocity);
    this->sendGeneralPacket(packet);

    this->debugLog("State pushed to ODrive module");

    return true;
}

bool ODriveModule::pullState() {
    // Pull the current state of the module from the physical module. Used to recover a ROS node
    // restart when the module is still running

    this->debugLog("Pulling state from ODrive module");

    // Request the control mode
    ROIPackets::Packet packet = ROIPackets::Packet();
    packet.setClientAddressOctet(this->getOctet());

    packet.setActionCode(ODriveConstants::GETCONTROLMODE);
    this->sendGeneralPacket(packet);

    // Request the input mode
    packet.setActionCode(ODriveConstants::GETINPUTMODE);
    this->sendGeneralPacket(packet);

    // Request the input torque
    packet.setActionCode(ODriveConstants::GETTORQUESETPOINT);
    this->sendGeneralPacket(packet);

    // Request the input position
    packet.setActionCode(ODriveConstants::GETPOSITIONSETPOINT);
    this->sendGeneralPacket(packet);

    // Request the input velocity
    packet.setActionCode(ODriveConstants::GETVELOCITYSETPOINT);
    this->sendGeneralPacket(packet);

    // Get all the non-state data
    packet.setActionCode(ODriveConstants::GETALL);
    this->sendGeneralPacket(packet);

    this->debugLog("State pulled from ODrive module");

    return true;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ODriveModule>());
    rclcpp::shutdown();
    return 0;
}

// python users fear the chad 1000 line .cpp file