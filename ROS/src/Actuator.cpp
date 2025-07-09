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
            if (!data[0]) {
                this->debugLog("Failed to set Actuator control");
            }
            break;

        case ActuatorConstants::SET_RELATIVE_LENGTH:
            if (!data[0]) {
                this->debugLog("Set_Relative_Length failed");
            }
            break;

        case ActuatorConstants::SET_VELOCITY:
            if (!data[0]) {
                this->debugLog("Set_Relative_Length failed");
            }
            break;

        case ActuatorConstants::GET_CURRENT_VELOCITY:
            _velocity = floatCast::toFloat(data, 0, 3);

            this->publishStateMessage();
            break;

        case ActuatorConstants::GET_CURRENT_LENGTH:
            _position = data[0] << 8 | data[1];

            this->publishStateMessage();
            break;

        default:
            this->debugLog("Unknown get action code received: " +
                           std::to_string(packet.getActionCode()));
            break;
    }

    // this->debugLog("Response handled");
}

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

    // Send the position set point
    ROIPackets::Packet packet = ROIPackets::Packet();
    packet.setClientAddressOctet(this->getOctet());
    packet.setSubDeviceID(sub_device_id);
    packet.setActionCode(ActuatorConstants::SET_ABSOLUTE_LENGTH);
    packet.setData_impSplit(position);
    packet.setSubDeviceID(sub_device_id);

    this->sendGeneralPacket(packet);
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

    // Send the position set point
    ROIPackets::Packet packet = ROIPackets::Packet();
    packet.setClientAddressOctet(this->getOctet());

    packet.setActionCode(ActuatorConstants::SET_RELATIVE_LENGTH);
    packet.setData_impSplit(position);
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
    packet.setData_impFloatCast(velocity);
    packet.setSubDeviceID(sub_device_id);

    this->sendGeneralPacket(packet);
    // this->debuglog(packet);
    // this->debugLog("Actuator Set Velocity Packet Called");
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
    // Resize the vectors to the number of actuators
    uint8_t actuatorCount =
        this->get_parameter("actuator_count").get_parameter_value().get<uint8_t>();
    this->_controlModes.resize(actuatorCount, ActuatorConstants::LENGTH_MODE);
    this->_inputPositions.resize(actuatorCount, 0);
    this->_relativeStartPositions.resize(actuatorCount, 0);
    this->_inputVelocities.resize(actuatorCount, 0);
    this->_positions.resize(actuatorCount, 0);
    this->_velocities.resize(actuatorCount, 0);

    // Create the state publishers
    this->_state_publishers_.clear();
    this->_publishStateMessages.clear();

    for (uint8_t i = 0;
         i < this->get_parameter("actuator_count").get_parameter_value().get<uint8_t>(); i++) {
        this->_state_publishers_.push_back(this->create_publisher<sensor_msgs::msg::JointState>(
            "roi_ros/act/axis" + std::to_string(i) + "/state", 10));

        // MSG ---------------
        //  State Publishers
        // Create lambda state publisher functions
        this->_publishStateMessages.push_back(
            [=]() {  //[=] captures the "i" for each func. "this" is implicitly passed as a pointer
                // Publish the state message, position and velocity
                sensor_msgs::msg::JointState message = sensor_msgs::msg::JointState();
                message.name.push_back("axis" + std::to_string(i));
                message.position[0] = _positions[i] / 1000.0;   // Convert to meters
                message.velocity[0] = _velocities[i] / 1000.0;  // Convert to m/s
                this->_state_publishers_[i]->publish(message);
            });  // Add a lambda function to the vector to publish the state message
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

    _parameterTimer = this->create_wall_timer(
        std::chrono::milliseconds(WatchdogConstants::MAINTAIN_SLEEP_TIME * 15),
        std::bind(&ActuatorModule::actuatorParameterCheck, this));

    // Send a status report to check module state
    ROIPackets::sysAdminPacket statusPacket = ROIPackets::sysAdminPacket();
    statusPacket.setAdminMetaData(sysAdminConstants::NO_CHAIN_META);
    statusPacket.setActionCode(sysAdminConstants::STATUS_REPORT);
    statusPacket.setClientAddressOctet(this->getOctet());

    this->sendSysadminPacket(statusPacket);
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

    this->debugLog("State pushed to Actuator module");

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
    // packet.setActionCode(ActuatorConstants::GET_ALL);
    // this->sendGeneralPacket(packet);

    // this->debugLog("State pulled from Actuator module");

    return true;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActuatorModule>());
    rclcpp::shutdown();
    return 0;
}
