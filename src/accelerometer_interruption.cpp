#include <iostream>
#include <wiringPi.h>
#include "MPU9250.h"

// Define the GPIO pin connected to the MPU9250 INT pin
// Using wiringPi pin numbers. Pin 0 is GPIO 17.
#define INTERRUPT_PIN 0

// A global flag to be set by the ISR
// 'volatile' is crucial here to ensure the compiler doesn't optimize away checks on this variable.
volatile bool motionDetected = false;

// The Interrupt Service Routine (ISR)
// This function is called automatically when the interrupt pin goes high.
// It should be as short and fast as possible.
void motionISR() {
    motionDetected = true;
}

int main() {
    // Create an instance of the MPU9250 class
    MPU9250 mpu;

    // Initialize the MPU9250
    if (!mpu.init()) {
        std::cerr << "Failed to initialize MPU9250." << std::endl;
        return -1;
    }

    // --- Setup the Interrupt ---
    // Set the GPIO pin as an input
    pinMode(INTERRUPT_PIN, INPUT);
    // Attach our ISR function to the pin's rising edge
    if (wiringPiISR(INTERRUPT_PIN, INT_EDGE_RISING, &motionISR) < 0) {
        std::cerr << "Unable to setup ISR." << std::endl;
        return 1;
    }

    // Enable Wake-on-Motion on the MPU9250 with a threshold of 200mg
    mpu.enableWakeOnMotion(200.0f);

    std::cout << "Waiting for motion..." << std::endl;

    // Main loop
    while (1) {
        // The program will spend most of its time in this delay, consuming very little CPU.
        if (motionDetected) {
            std::cout << "\nMotion Detected!" << std::endl;

            // IMPORTANT: Read the interrupt status register to clear the interrupt pin.
            // If you don't do this, the interrupt will stay active and you won't get new ones.
            uint8_t status = mpu.getInterruptStatus();

            // You can optionally print the status to see which interrupt was triggered
            std::cout << "Interrupt Status: 0x" << std::hex << (int)status << std::dec << std::endl;
            
            // Now you can read the sensor data to see the motion
            mpu.update();
            std::cout << "Accel [g]: X=" << mpu.ax << ", Y=" << mpu.ay << ", Z=" << mpu.az << std::endl;

            // Reset the flag and wait for the next motion
            motionDetected = false;
            std::cout << "\nWaiting for motion..." << std::endl;
        }

        delay(100); // Check the flag every 100ms
    }

    return 0;
}

