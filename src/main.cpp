#include <iostream>
#include <iomanip> // For std::fixed and std::setprecision
#include "MPU9250.h"
#include <wiringPi.h>

int main(void) {
    // Create an instance of the MPU9250 class
    MPU9250 mpu;

    // Initialize the MPU9250 with default settings
    // AFS_2G, GFS_250DPS, MFS_16BITS, M_100Hz_CONTINUOUS
    if (!mpu.init()) {
        std::cerr << "Failed to initialize MPU9250. Please check connections." << std::endl;
        return -1;
    }

    // Calibrate the accelerometer and gyroscope
    // Keep the sensor level and motionless during this process
    mpu.calibrate();

    // Main loop to read and display data
    while (1) {
        // Read the latest data from all sensors
        mpu.update();

        // Set output formatting
        std::cout << std::fixed << std::setprecision(3);

        // Print Accelerometer data
        std::cout << "Accel [g]:  "
                  << "X=" << std::setw(6) << mpu.ax << " | "
                  << "Y=" << std::setw(6) << mpu.ay << " | "
                  << "Z=" << std::setw(6) << mpu.az << std::endl;

        // Print Gyroscope data
        std::cout << "Gyro  [dps]:"
                  << "X=" << std::setw(6) << mpu.gx << " | "
                  << "Y=" << std::setw(6) << mpu.gy << " | "
                  << "Z=" << std::setw(6) << mpu.gz << std::endl;

        // Print Magnetometer data
        std::cout << "Mag   [mG]: "
                  << "X=" << std::setw(6) << mpu.mx << " | "
                  << "Y=" << std::setw(6) << mpu.my << " | "
                  << "Z=" << std::setw(6) << mpu.mz << std::endl;

        // Print Temperature
        std::cout << "Temp  [C]:  " << mpu.temperature << std::endl;

        std::cout << "--------------------------------------------------" << std::endl;

        // Delay for 500 milliseconds (2 Hz update rate)
        delay(50);
    }

    return 0;
}

