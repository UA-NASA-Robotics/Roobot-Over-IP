#ifndef MOTOR_DRIVER_PID
#define MOTOR_DRIVER_PID

#include <stdint.h>

struct PidTuning {
    uint8_t kp = 0;
    uint8_t ki = 0;
    uint8_t kd = 0;
};

class MotorDriverPid {
   private:
    PidTuning _tuning;

   public:
};

#endif