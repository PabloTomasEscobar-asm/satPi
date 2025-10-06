#include <iostream>
#include <cmath> // Required for abs()
#include <iomanip> // Required for std::fixed, std::setprecision
#include <wiringPi.h>
#include "MPU9250.h"

int main() {
    // Create an instance of the MPU9250 class
    MPU9250 mpu;

    // Initialize the MPU9250.
    // It's important to set a scale that includes the threshold you want to detect.
    // AFS_2G is perfect for detecting 1g.
    if (!mpu.init(AFS_2G)) {
        std::cerr << "Failed to initialize MPU9250." << std::endl;
        return -1;
    }

    // Calibrate the sensor for more accurate readings
    mpu.calibrate();

    std::cout << "Polling for motion on Y-axis greater than 1g..." << std::endl;

    // Main loop for software polling
    while (1) {
        // Always read the latest sensor data in the loop
        mpu.update();

        // Check if the absolute value of Y-axis acceleration is greater than 1.0g
        if (std::abs(mpu.az) > 1.0f) {
            std::cout << "\nMotion Detected on z-axis!" << std::endl;
            
            // Print the value that triggered the detection
            std::cout << std::fixed << std::setprecision(3);
            std::cout << "z-axis acceleration: " << mpu.az << " g" << std::endl;
            
            // Wait for 2 seconds before continuing to avoid spamming the console
            delay(2000); 

            std::cout << "\nPolling for motion..." << std::endl;
        }

        // Wait for a short period before the next check to avoid maxing out the CPU
        delay(100); 
    }

    return 0;
}


