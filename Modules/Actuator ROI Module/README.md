# Actuator Module

did i mention we are controlling 2 actuators? Hope your code is scalable.

### Hardware

Arduino Pins:

-   D9 => ACT 1 PWM (Speed)
-   D8 => ACT 1 DIR (Direction)
-   D7 => ACT 2 DIR (Direction)
-   D6 => ACT 2 PWM (Speed)

-   D5 => Encoder Register Load Signal (Active Low)
-   D4 => Encoder Register Clock Signal
-   D3 => Encoder Register 1 Clear Signal (Active Low)
-   D2 <= Encoder Register 1 Shift In Signal
-   A0 => Encoder Register 2 Clear Signal (Active Low)
-   A1 <= Encoder Register 2 Shift In Signal

-   A2 <= ACT 1 Retract Limit Switch (Internal Pullup + active low)
-   A3 <= ACT 1 Extend Limit Switch (Internal Pullup + active low)
-   A4 <= ACT 2 Retract Limit Switch (Internal Pullup + active low)
-   A5 <= ACT 2 Extend Limit Switch (Internal Pullup + active low)

### Signaling

#### Reset Encoder Counter

Sets the encoder counter to zero, asynchronously. No other operations required.

#### Read Sequence

1. Set the Encoder Register Load Signal to LOW
2. Clock a rising edge on the Encoder Register Clock Signal. (This will load the encoder registers unless the counter propagation delay is blocking the load signal. To be improved, but if you read in all 0s and the last reading was far from 0, try again.)
3. Set the Encoder Register Load Signal to HIGH
4. Clock a rising edge on the Encoder Register Clock Signal
5. Read the value of the Encoder Register 1 Shift In Signal
6. Read the value of the Encoder Register 2 Shift In Signal
7. Repeat 4-6 15 more times.

Note you always load and clock both actuator encoder registers at the same time.
