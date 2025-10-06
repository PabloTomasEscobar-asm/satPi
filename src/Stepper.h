}#ifndef ULN2003STEPPER_H
#define ULN2003STEPPER_H

#include <vector>

class ULN2003Stepper {
public:
    /**
     * @brief Constructor for the ULN2003Stepper class.
     * @param pin1 GPIO pin connected to IN1 of the ULN2003.
     * @param pin2 GPIO pin connected to IN2 of the ULN2003.
     * @param pin3 GPIO pin connected to IN3 of the ULN2003.
     * @param pin4 GPIO pin connected to IN4 of the ULN2003.
     */
    ULN2003Stepper(int pin1, int pin2, int pin3, int pin4);

    /**
     * @brief Sets the motor speed in rotations per minute (RPM).
     * @param rpm The desired speed in RPM.
     */
    void setSpeed(long rpm);

    /**
     * @brief Moves the motor a specific number of steps.
     * @param steps The number of steps to move. Positive for forward, negative for backward.
     */
    void step(int steps);

    /**
     * @brief Stops the motor and turns off all coils to save power.
     */
    void stop();

private:
    void stepMotor(int thisStep);

    std::vector<int> pins;
    int currentStep;
    long step_delay; // in microseconds
    int steps_per_revolution;

    // Step sequence for full-step mode
    const int step_sequence[4][4] = {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };
};

#endif // ULN2003STEPPER_H
