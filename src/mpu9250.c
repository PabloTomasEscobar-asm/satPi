#include "mpu9250.h"
#include <stdio.h>
#include <errno.h>
#include <string.h>

/**
 * @brief Initializes the MPU-9250 sensor.
 *
 * @param devId The I2C address of the device.
 * @return The file descriptor on success, -1 on error.
 */
int mpu9250_init(int devId) {
    int fd = wiringPiI2CSetup(devId);
    if (fd == -1) {
        fprintf(stderr, "Failed to initialize I2C device at address 0x%x. Error: %s\\n", devId, strerror(errno));
        return -1;
    }

    // Check device ID
    int who_am_i = wiringPiI2CReadReg8(fd, WHO_AM_I);
    
    // Some MPU-9250 versions return 0x73, so we'll accept both 0x71 and 0x73.
    if (who_am_i != 0x71 && who_am_i != 0x73) {
        fprintf(stderr, "Incorrect device ID. Expected 0x71 or 0x73, but got 0x%x\\n", who_am_i);
        return -1;
    }

    printf("MPU-9250 found with device ID 0x%x\\n", who_am_i);

    // Wake up the MPU-9250
    wiringPiI2CWriteReg8(fd, PWR_MGMT_1, 0x00);

    // Configure gyroscope and accelerometer
    wiringPiI2CWriteReg8(fd, GYRO_CONFIG, 0x08);  // Gyroscope range: ±500 dps
    wiringPiI2CWriteReg8(fd, ACCEL_CONFIG, 0x08); // Accelerometer range: ±4g

    return fd;
}

/**
 * @brief Reads a 16-bit word (two bytes) from a register.
 *
 * @param fd The file descriptor for the I2C device.
 * @param reg The register address to read from.
 * @return The 16-bit value read.
 */
int16_t mpu9250_read_word(int fd, int reg) {
    int high = wiringPiI2CReadReg8(fd, reg);
    int low = wiringPiI2CReadReg8(fd, reg + 1);
    int16_t value = (high << 8) | low;
    return value;
}

/**
 * @brief Reads the accelerometer data.
 *
 * @param fd The file descriptor for the I2C device.
 * @param ax Pointer to store the X-axis value.
 * @param ay Pointer to store the Y-axis value.
 * @param az Pointer to store the Z-axis value.
 */
void mpu9250_read_accel(int fd, int16_t* ax, int16_t* ay, int16_t* az) {
    *ax = mpu9250_read_word(fd, ACCEL_XOUT_H);
    *ay = mpu9250_read_word(fd, ACCEL_YOUT_H);
    *az = mpu9250_read_word(fd, ACCEL_ZOUT_H);
}

/**
 * @brief Reads the gyroscope data.
 *
 * @param fd The file descriptor for the I2C device.
 * @param gx Pointer to store the X-axis value.
 * @param gy Pointer to store the Y-axis value.
 * @param gz Pointer to store the Z-axis value.
 */
void mpu9250_read_gyro(int fd, int16_t* gx, int16_t* gy, int16_t* gz) {
    *gx = mpu9250_read_word(fd, GYRO_XOUT_H);
    *gy = mpu9250_read_word(fd, GYRO_YOUT_H);
    *gz = mpu9250_read_word(fd, GYRO_ZOUT_H);
}
