#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "../../../../../../lib/UDP-API/actuator.h"
#include "../../../../../../lib/UDP-API/packetTypes.h"
#include "EncoderDrivers/EncoderDriverBase.h"
#include "LimitSwitch.h"
#include "MotorDrivers/MotorDriverBase.h"

class Actuator {
   private:
    bool _homed = false;        // Flag to check if actuator was homed
    bool _initialized = false;  // Flag to check if actuator was initialized
    bool _control_mode =        // Control mode flag (CMF)
        ActuatorConstants::VELOCITY_MODE;
    const uint8_t _LIMIT_STATE;  // Limit switch state

    const uint16_t _MAX_LENGTH;  // Maximum length the actuator should extend
    const uint16_t _MIN_LENGTH;  // Minimum length the actuator should extend

    float _velocity = 0;   // Target velocity (CMF == VEL_CONTROL)
    uint16_t _length = 0;  // Target length (CMF == LEN_CONTROL)

    uint32_t _homed_time = -1;  // Time the actuator was last homed in ms

    MotorDriverBase* _motor;    // Motor driver
    EncoderDriverBase* _enc;    // Encoder
    LimitSwitch* _upper_limit;  // Upper limit switch
    LimitSwitch* _lower_limit;  // Lower limit switch

    // Returns if a limit switch is active or not
    bool _limitSwitchActivated(uint8_t state = 0xFF);

    // Passively home the actuator if at min extension
    void _passiveHome();

   public:
    /**
     * @brief Actuator class constructor
     *
     * @param enc    Pointer to a derived encoder driver for the actuator
     * @param motor  Pointer to a derived motor driver for the actuator
     * @param upper  Pointer to the upper-extension limit switch for the actuator
     * @param lower  Pointer to the lower-extention limit switch for the actuator
     * @param min_length The minimum extention limit of the actuator
     * @param max_length The maximum extention limit of the actuator
     */
    Actuator(EncoderDriverBase* enc, MotorDriverBase* motor, LimitSwitch* upper, LimitSwitch* lower,
             uint16_t min_length, uint16_t max_length);

    /**
     * @brief Actuator class constructor
     *
     * @param enc            Pointer to a derived encoder driver for the actuator
     * @param motor          Pointer to a derived motor driver for the actuator
     * @param limit_switch   Pointer to the limit switch for the actuator
     * @param limit_state    The limit switch's location relative to the actuator (UPPER_LIMIT or
     * LOWER_LIMIT)
     */
    Actuator(EncoderDriverBase* enc, MotorDriverBase* motor, LimitSwitch* limit_switch,
             uint8_t limit_state, uint16_t min_length, uint16_t max_length);

    /**
     * @brief Actuator class constructor
     *
     * @param enc    Pointer to a derived encoder driver for the actuator
     * @param motor  Pointer to a derived motor driver for the actuator
     */
    Actuator(EncoderDriverBase* enc, MotorDriverBase* motor, uint16_t min_length,
             uint16_t max_length);

    /**
     * @brief Initialize the actuator and it's components
     */
    void init();

    /**
     * @brief Run a tick on the actuator's motor control loop
     */
    void tick();

    /**
     * @brief Set a target velocity for the actuator's motor
     *
     * @param vel   The desired target velocity for the actuator's motor in RPM
     */
    void setVelocity(float vel);

    /**
     * @brief Update target length by a relative position in mm
     *
     * @param rel_len   The desired relative length for the actuator's motor in mm
     */
    void setRelativeLength(uint16_t len);

    /**
     * @brief Update target length to an absolute position in mm
     *
     * @param abs_len   The desired absolute length for the actuator's motor in mm
     */
    void setAbsoluteLength(uint16_t len);

    /**
     * @brief Set the desired control mode
     *
     * @param control_mode  The desired control mode
     */
    void setControlMode(payloadConstant control_mode);
};

#endif