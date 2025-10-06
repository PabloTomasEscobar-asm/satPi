#include "Stepper.h"
#include <wiringPi.h>

ULN2003Stepper::ULN2003Stepper(int pin1, int pin2, int pin3, int pin4) {
    pins.push_back(pin1);
    pins.push_back(pin2);
    pins.push_back(pin3);
    pins.push_back(pin4);

    currentStep = 0;
    steps_per_revolution = 2048;

    for (int pin : pins) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
    setSpeed(10); 
}

void ULN2003Stepper::setSpeed(long rpm) {
    if (rpm > 0) {
        step_delay = 60L * 1000L * 1000L / steps_per_revolution / rpm;
    }
}

void ULN2003Stepper::step(int steps) {
    int steps_left = abs(steps);

    while (steps_left > 0) {
        if (steps > 0) {
            currentStep++;
            if (currentStep >= 4) {
                currentStep = 0;
            }
        } else {
            currentStep--;
            if (currentStep < 0) {
                currentStep = 3;
            }
        }
        stepMotor(currentStep);
        delayMicroseconds(step_delay);
        steps_left--;
    }
}

void ULN2003Stepper::stepMotor(int thisStep) {
    for (int i = 0; i < 4; i++) {
        digitalWrite(pins[i], step_sequence[thisStep][i]);
    }
}

void ULN2003Stepper::stop() {
    for (int pin : pins) {
        digitalWrite(pin, LOW);
    }
}

