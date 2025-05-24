#ifndef IBT2_BINARY_MOTOR_H
#define IBT2_BINARY_MOTOR_H

#include <stdint.h>

#include "MotorDriverBase.h"

class IBT2BinaryMotor : public MotorDriverBase {
   private:
    const uint8_t _FWD_PIN, _BCK_PIN;  // H-bridge pins do drive forward/backward

   public:
    IBT2BinaryMotor(uint8_t fwd_pin, uint8_t bck_pin);

    /**
     * @brief Returns the max speed of the motor in mm/s
     *
     * @return The max speed of the motor in mm/s
     */
    uint8_t maxSpeed() override;

    /**
     * @brief Initialize the Arduino pins for this motor
     */
    void init() override;

    /**
     * @brief Update the motor's speed
     */
    void tick(EncoderDriverBase* enc, uint16_t target_length,
              payloadConstant control_mode) override;
};

#endif