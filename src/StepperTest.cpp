#include <iostream>
#include <wiringPi.h>
#include "ULN2003Stepper.h"

#define IN1 0   //Board pin 11 
#define IN2 2   //Board pin 13
#define IN3 3   //Board pin 15
#define IN4 4   //Board pin 16

int main() {
    if (wiringPiSetup() == -1) {
        std::cerr << "Error al inicializar wiringPi." << std::endl;
        return 1;
    }

    std::cout << "Control de Motor Paso a Paso con ULN2003" << std::endl;

    ULN2003Stepper myStepper(IN1, IN2, IN3, IN4);

    myStepper.setSpeed(15);

    std::cout << "Girando indefinidamente en sentido horario..." << std::endl;
    std::cout << "Presiona Ctrl+C para detener." << std::endl;

    while (true) {
        myStepper.step(1);
    }
    myStepper.stop();

    return 0;
}


