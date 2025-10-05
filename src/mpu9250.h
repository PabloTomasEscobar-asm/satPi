#ifndef MPU9250_H
#define MPU9250_H

#include <wiringPiI2C.h>
#include <stdint.h>

#define MPU9250_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H  0x43
#define GYRO_YOUT_H  0x45
#define GYRO_ZOUT_H  0x47
#define WHO_AM_I     0x71 //0x68 for the mpu6050

int mpu9250_init(int devId);
int16_t mpu9250_read_word(int fd, int reg);
void mpu9250_read_accel(int fd, int16_t* ax, int16_t* ay, int16_t* az);
void mpu9250_read_gyro(int fd, int16_t* gx, int16_t* gy, int16_t* gz);

#endif // MPU9250_H
