#include "Actuator.h"

#define __min(a, b) (((a) < (b)) ? (a) : (b))  // idk why i had to define this myself

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
    ROIPackets::Packet readPacket = ROIPackets::Packet();
    readPacket.setActionCode(ActuatorConstants::GET_CURRENT_VELOCITY);
    this->sendGeneralPacket(readPacket);
    readPacket.setActionCode(ActuatorConstants::GET_CURRENT_LENGTH);
    this->sendGeneralPacket(readPacket);
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
    // Handle the response to a get request
    switch (packet.getActionCode()) {
        case ActuatorConstants::GET_CONTROL:
            _controlMode = data[0];
            break;
        
        case ActuatorConstants::SET_CONTROL:
            if(!data[0]){this->debugLog("Failed to set Actuator control");}
            break;
        

        case ActuatorConstants::SET_RELATIVE_LENGTH:
            if(!data[0]){this->debugLog("Set_Relative_Length failed");}
            break;

        case ActuatorConstants::SET_VELOCITY:
            if(!data[0]){this->debugLog("Set_Relative_Length failed");}
            break;

        case ActuatorConstants::GET_CURRENT_VELOCITY:
            _velocity = floatCast::toFloat(data, 0, 3);

            this->publishStateMessage();
            break;

        case ActuatorConstants::GET_CURRENT_LENGTH:
            _position = data[0]<<8 | data[1];

            this->publishStateMessage();
            break;


        default:
            this->debugLog("Unknown get action code received: " +
                            std::to_string(packet.getActionCode()));
            break;
    }

    // this->debugLog("Response handled");
}


void ActuatorModule::publishStateMessage() {
    // Publish the state message, position and velocity
    auto message = roi_ros::msg::ActuatorState();
    message.position = _position;
    message.velocity = _velocity;
    this->_state_publisher_->publish(message);
}

void ActuatorModule::gotoAbsolutePositionServiceHandler(
    const roi_ros::srv::ActuatorGotoAbsolutePosition::Request::SharedPtr request,
    roi_ros::srv::ActuatorGotoAbsolutePosition::Response::SharedPtr response) {
    // Handle the goto position service request
    this->debugLog("Received goto position service request");

    this->sendGotoAbsolutePositionPacket(request->position, request->sub_device_id);

    // Respond to the service request
    response->success = !_healthData._module_error;  // if there is an error, success is false, we
                                                     // may have not done the request
    this->debugLog("Goto position service request handled");
}

void ActuatorModule::gotoRelativePositionServiceHandler(
    const roi_ros::srv::ActuatorGotoRelativePosition::Request::SharedPtr request,
    roi_ros::srv::ActuatorGotoRelativePosition::Response::SharedPtr response) {
    // Handle the goto position service request
    this->debugLog("Received goto position service request");

    this->sendGotoRelativePositionPacket(request->position, request->sub_device_id);

    // Respond to the service request
    response->success = !_healthData._module_error;  // if there is an error, success is false, we
                                                     // may have not done the request
    this->debugLog("Goto position service request handled");
}

void ActuatorModule::setVelocityServiceHandler(
    const roi_ros::srv::ActuatorSetVelocity::Request::SharedPtr request,
    roi_ros::srv::ActuatorSetVelocity::Response::SharedPtr response) {
    // Handle the set velocity service request
    // this->debugLog("Received set velocity service request");

    this->sendSetVelocityPacket(request->velocity, request->sub_device_id);

    // Respond to the service request
    response->success = !_healthData._module_error;

    // this->debugLog("Set velocity service request handled");
}

void ActuatorModule::sendGotoAbsolutePositionPacket(uint16_t position, uint16_t sub_device_id) {
    // Set the Actuator to position mode if needed to complete request
    if (_controlMode != ActuatorConstants::LENGTH_MODE) {
        ROIPackets::Packet packet = ROIPackets::Packet();
        packet.setClientAddressOctet(this->getOctet());
        packet.setActionCode(ActuatorConstants::SET_CONTROL);
        packet.setData(ActuatorConstants::LENGTH_MODE);

        this->sendGeneralPacket(packet);

        _controlMode = ActuatorConstants::LENGTH_MODE;
    }

    // Send the position set point
    ROIPackets::Packet packet = ROIPackets::Packet();
    packet.setClientAddressOctet(this->getOctet());
    packet.setActionCode(ActuatorConstants::SET_ABSOLUTE_LENGTH);
    packet.setData_impSplit(position);
    packet.setSubDeviceID(sub_device_id);

    this->sendGeneralPacket(packet);
}

void ActuatorModule::sendGotoRelativePositionPacket(uint16_t position, uint16_t sub_device_id) {
    // Set the Actuator to position mode if needed to complete request
    if (_controlMode != ActuatorConstants::LENGTH_MODE) {
        ROIPackets::Packet packet = ROIPackets::Packet();
        packet.setClientAddressOctet(this->getOctet());
        packet.setActionCode(ActuatorConstants::SET_CONTROL);
        packet.setData(ActuatorConstants::LENGTH_MODE);

        this->sendGeneralPacket(packet);

        _controlMode = ActuatorConstants::LENGTH_MODE;
    }

    // Send the position set point
    ROIPackets::Packet packet = ROIPackets::Packet();
    packet.setClientAddressOctet(this->getOctet());
    packet.setActionCode(ActuatorConstants::SET_RELATIVE_LENGTH);
    packet.setData_impSpilt(position);
    packet.setSubDeviceID(sub_device_id);

    this->sendGeneralPacket(packet);
}

void ActuatorModule::sendSetVelocityPacket(float velocity, uint16_t sub_device_id) {
    if (_controlMode != ActuatorConstants::VELOCITY_MODE) {
        ROIPackets::Packet packet = ROIPackets::Packet();
        packet.setClientAddressOctet(this->getOctet());
        packet.setActionCode(ActuatorConstants::SET_CONTROL);
        packet.setData(ActuatorConstants::VELOCITY_MODE);

        this->sendGeneralPacket(packet);
        _controlMode = ActuatorConstants::VELOCITY_MODE;
    }

    ROIPackets::Packet packet = ROIPackets::Packet();
    packet.setClientAddressOctet(this->getOctet());
    packet.setActionCode(ActuatorConstants::SET_VELOCITY);
    packet.setData_impCast(velocity);
    packet.setSubDeviceID(sub_device_id);

    this->sendGeneralPacket(packet);
}

//-------- PUBLIC METHODS --------//

ActuatorModule::ActuatorModule() : BaseModule("ActuatorModule", moduleTypesConstants::ACTUATOR) {
    // Initialize the Actuator module
    // this->debugLog("Initializing Actuator Module");

    this->_state_publisher_ = this->create_publisher<roi_ros::msg::ActuatorState>("state", 10);

    // Initialize the Actuator specific services
    this->_goto_position_service_ = this->create_service<roi_ros::srv::ActuatorGotoAbsolutePosition>(
        "goto_position", std::bind(&ActuatorModule::gotoAbsolutePositionServiceHandler, this,
                                   std::placeholders::_1, std::placeholders::_2));

    this->_goto_relative_position_service_ =
        this->create_service<roi_ros::srv::ActuatorGotoRelativePosition>(
            "goto_relative_position",
            std::bind(&ActuatorModule::gotoRelativePositionServiceHandler, this,
                      std::placeholders::_1, std::placeholders::_2));

    this->_set_velocity_service_ = this->create_service<roi_ros::srv::ActuatorSetVelocity>(
        "set_velocity", std::bind(&ActuatorModule::setVelocityServiceHandler, this,
                                  std::placeholders::_1, std::placeholders::_2));
}

ActuatorModule::~ActuatorModule(){
    // Destroy the GPIO module
    this->debugLog("Destroying Actuator Module");
}

bool ActuatorModule::pushState() {
    // Push the current stored state of the module to the physical module
    // This is used to recover the state of the module after a hardware reset but Ros node still
    // alive.

    this->debugLog("Pushing state to Actuator module");

    // Push the control mode
    ROIPackets::Packet packet = ROIPackets::Packet();
    packet.setClientAddressOctet(this->getOctet());
    packet.setActionCode(ActuatorConstants::SET_CONTROL);
    packet.setData(_controlMode);
    this->sendGeneralPacket(packet);

    // Push the input position
    packet.setActionCode(ActuatorConstants::SET_ABSOLUTE_LENGTH);
    packet.setData(_inputPosition);
    this->sendGeneralPacket(packet);

    // Push the input velocity
    packet.setActionCode(ActuatorConstants::SET_VELOCITY);
    packet.setData(_inputVelocity);
    this->sendGeneralPacket(packet);

    // this->debugLog("State pushed to Actuator module");

    return true;
}

bool ActuatorModule::pullState() {
    // Pull the current state of the module from the physical module. Used to recover a ROS node
    // restart when the module is still running

    this->debugLog("Pulling state from Actuator module");

    // Request the control mode
    ROIPackets::Packet packet = ROIPackets::Packet();
    packet.setClientAddressOctet(this->getOctet());

    packet.setActionCode(ActuatorConstants::GET_CONTROL);
    this->sendGeneralPacket(packet);

    // Request the input position
    packet.setActionCode(ActuatorConstants::GET_TARGET_LENGTH);
    this->sendGeneralPacket(packet);

    // Request the input velocity
    packet.setActionCode(ActuatorConstants::GET_TARGET_VELOCITY);
    this->sendGeneralPacket(packet);

    // Get all the non-state data
    //packet.setActionCode(ActuatorConstants::GET_ALL);
    //this->sendGeneralPacket(packet);

    // this->debugLog("State pulled from Actuator module");

    return true;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActuatorModule>());
    rclcpp::shutdown();
    return 0;
}

// python users fear the chad 1000 line .cpp file