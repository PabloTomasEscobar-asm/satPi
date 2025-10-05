#pragma once
#ifndef MPU9250_H
#define MPU9250_H

#include <wiringPiI2C.h>
#include <stdint.h>
// MPU9250 and AK8963 Register Map

// Note: The MPU9250 is a 9-axis motion tracking device that combines a 3-axis gyroscope, 
// 3-axis accelerometer, 3-axis magnetometer, and a Digital Motion Processor™ (DMP).
// The magnetometer is an Asahi Kasei Microdevices Corporation AK8963.

// ===============================================================================
//                              AK8963 Magnetometer Registers
// ===============================================================================
#define AK8963_ADDRESS   0x0C   // I2C address of the magnetometer
#define AK8963_WHO_AM_I  0x00   // (R) should return 0x48
#define AK8963_INFO      0x01   // (R) Information
#define AK8963_ST1       0x02   // (R) Data ready status bit 0
#define AK8963_XOUT_L    0x03   // (R) X-axis magnetometer data low byte
#define AK8963_XOUT_H    0x04   // (R) X-axis magnetometer data high byte
#define AK8963_YOUT_L    0x05   // (R) Y-axis magnetometer data low byte
#define AK8963_YOUT_H    0x06   // (R) Y-axis magnetometer data high byte
#define AK8963_ZOUT_L    0x07   // (R) Z-axis magnetometer data low byte
#define AK8963_ZOUT_H    0x08   // (R) Z-axis magnetometer data high byte
#define AK8963_ST2       0x09   // (R) Data overflow and data read error status
#define AK8963_CNTL      0x0A   // (R/W) Control register for modes
#define AK8963_ASTC      0x0C   // (R/W) Self-test control
#define AK8963_I2CDIS    0x0F   // (R/W) I2C disable
#define AK8963_ASAX      0x10   // (R) Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11   // (R) Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12   // (R) Fuse ROM z-axis sensitivity adjustment value

// ===============================================================================
//                              MPU9250 Self-Test Registers
// ===============================================================================
#define SELF_TEST_X_GYRO  0x00
#define SELF_TEST_Y_GYRO  0x01
#define SELF_TEST_Z_GYRO  0x02
#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F
#define SELF_TEST_A       0x10

// ===============================================================================
//                              MPU9250 Offset Registers
// ===============================================================================
#define XG_OFFSET_H      0x13  // (R/W) User-defined trim values for gyroscope X-axis high byte
#define XG_OFFSET_L      0x14  // (R/W) User-defined trim values for gyroscope X-axis low byte
#define YG_OFFSET_H      0x15  // (R/W) User-defined trim values for gyroscope Y-axis high byte
#define YG_OFFSET_L      0x16  // (R/W) User-defined trim values for gyroscope Y-axis low byte
#define ZG_OFFSET_H      0x17  // (R/W) User-defined trim values for gyroscope Z-axis high byte
#define ZG_OFFSET_L      0x18  // (R/W) User-defined trim values for gyroscope Z-axis low byte
#define XA_OFFSET_H      0x77  // (R/W) User-defined trim values for accelerometer X-axis high byte
#define XA_OFFSET_L      0x78  // (R/W) User-defined trim values for accelerometer X-axis low byte
#define YA_OFFSET_H      0x7A  // (R/W) User-defined trim values for accelerometer Y-axis high byte
#define YA_OFFSET_L      0x7B  // (R/W) User-defined trim values for accelerometer Y-axis low byte
#define ZA_OFFSET_H      0x7D  // (R/W) User-defined trim values for accelerometer Z-axis high byte
#define ZA_OFFSET_L      0x7E  // (R/W) User-defined trim values for accelerometer Z-axis low byte

// ===============================================================================
//                              MPU9250 Configuration Registers
// ===============================================================================
#define SMPLRT_DIV       0x19  // (R/W) Sample Rate Divider
#define CONFIG           0x1A  // (R/W) General configuration
#define GYRO_CONFIG      0x1B  // (R/W) Gyroscope configuration
#define ACCEL_CONFIG     0x1C  // (R/W) Accelerometer configuration
#define ACCEL_CONFIG2    0x1D  // (R/W) Accelerometer configuration 2
#define LP_ACCEL_ODR     0x1E  // (R/W) Low Power Accelerometer Output Data Rate
#define WOM_THR          0x1F  // (R/W) Wake-on-Motion Threshold

// ===============================================================================
//                              MPU9250 Motion Detection Registers
// ===============================================================================
#define MOT_DUR          0x20  // (R/W) Motion detection duration
#define ZMOT_THR         0x21  // (R/W) Zero-motion detection threshold
#define ZRMOT_DUR        0x22  // (R/W) Zero-motion detection duration
#define MOT_DETECT_CTRL  0x69  // (R/W) Motion detection control
#define MOT_DETECT_STATUS 0x61 // (R) Motion detection status

// ===============================================================================
//                              MPU9250 I2C Master/Slave Registers
// ===============================================================================
#define I2C_MST_CTRL     0x24  // (R/W) I2C Master Control
#define I2C_SLV0_ADDR    0x25  // (R/W) I2C Slave 0 Address
#define I2C_SLV0_REG     0x26  // (R/W) I2C Slave 0 Register
#define I2C_SLV0_CTRL    0x27  // (R/W) I2C Slave 0 Control
#define I2C_SLV1_ADDR    0x28  // (R/W) I2C Slave 1 Address
#define I2C_SLV1_REG     0x29  // (R/W) I2C Slave 1 Register
#define I2C_SLV1_CTRL    0x2A  // (R/W) I2C Slave 1 Control
#define I2C_SLV2_ADDR    0x2B  // (R/W) I2C Slave 2 Address
#define I2C_SLV2_REG     0x2C  // (R/W) I2C Slave 2 Register
#define I2C_SLV2_CTRL    0x2D  // (R/W) I2C Slave 2 Control
#define I2C_SLV3_ADDR    0x2E  // (R/W) I2C Slave 3 Address
#define I2C_SLV3_REG     0x2F  // (R/W) I2C Slave 3 Register
#define I2C_SLV3_CTRL    0x30  // (R/W) I2C Slave 3 Control
#define I2C_SLV4_ADDR    0x31  // (R/W) I2C Slave 4 Address
#define I2C_SLV4_REG     0x32  // (R/W) I2C Slave 4 Register
#define I2C_SLV4_DO      0x33  // (R/W) I2C Slave 4 Data Out
#define I2C_SLV4_CTRL    0x34  // (R/W) I2C Slave 4 Control
#define I2C_SLV4_DI      0x35  // (R) I2C Slave 4 Data In
#define I2C_MST_STATUS   0x36  // (R) I2C Master Status
#define I2C_SLV0_DO      0x63  // (R/W) I2C Slave 0 Data Out
#define I2C_SLV1_DO      0x64  // (R/W) I2C Slave 1 Data Out
#define I2C_SLV2_DO      0x65  // (R/W) I2C Slave 2 Data Out
#define I2C_SLV3_DO      0x66  // (R/W) I2C Slave 3 Data Out
#define I2C_MST_DELAY_CTRL 0x67// (R/W) I2C Master Delay Control

// ===============================================================================
//                              MPU9250 Interrupt Registers
// ===============================================================================
#define INT_PIN_CFG      0x37  // (R/W) Interrupt Pin / Bypass Enable Configuration
#define INT_ENABLE       0x38  // (R/W) Interrupt Enable
#define DMP_INT_STATUS   0x39  // (R) DMP Interrupt Status
#define INT_STATUS       0x3A  // (R) Interrupt Status

// ===============================================================================
//                              MPU9250 Sensor Data Output Registers
// ===============================================================================
#define ACCEL_XOUT_H     0x3B  // (R) Accelerometer Measurements X-axis high byte
#define ACCEL_XOUT_L     0x3C  // (R) Accelerometer Measurements X-axis low byte
#define ACCEL_YOUT_H     0x3D  // (R) Accelerometer Measurements Y-axis high byte
#define ACCEL_YOUT_L     0x3E  // (R) Accelerometer Measurements Y-axis low byte
#define ACCEL_ZOUT_H     0x3F  // (R) Accelerometer Measurements Z-axis high byte
#define ACCEL_ZOUT_L     0x40  // (R) Accelerometer Measurements Z-axis low byte
#define TEMP_OUT_H       0x41  // (R) Temperature Measurement high byte
#define TEMP_OUT_L       0x42  // (R) Temperature Measurement low byte
#define GYRO_XOUT_H      0x43  // (R) Gyroscope Measurements X-axis high byte
#define GYRO_XOUT_L      0x44  // (R) Gyroscope Measurements X-axis low byte
#define GYRO_YOUT_H      0x45  // (R) Gyroscope Measurements Y-axis high byte
#define GYRO_YOUT_L      0x46  // (R) Gyroscope Measurements Y-axis low byte
#define GYRO_ZOUT_H      0x47  // (R) Gyroscope Measurements Z-axis high byte
#define GYRO_ZOUT_L      0x48  // (R) Gyroscope Measurements Z-axis low byte

// ===============================================================================
//                              MPU9250 External Sensor Data Registers
// ===============================================================================
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60

// ===============================================================================
//                              MPU9250 FIFO Registers
// ===============================================================================
#define FIFO_EN          0x23  // (R/W) FIFO Enable
#define FIFO_COUNTH      0x72  // (R) FIFO Count high byte
#define FIFO_COUNTL      0x73  // (R) FIFO Count low byte
#define FIFO_R_W         0x74  // (R/W) FIFO Read Write

// ===============================================================================
//                              MPU9250 DMP Registers
// ===============================================================================
#define DMP_BANK         0x6D  // (R/W) Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // (R/W) Set read/write pointer to a specific start address
#define DMP_REG          0x6F  // (R/W) Register in DMP to read from or write to
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71 

// ===============================================================================
//                              MPU9250 Control and Power Registers
// ===============================================================================
#define SIGNAL_PATH_RESET  0x68  // (R/W) Signal Path Reset
#define USER_CTRL        0x6A  // (R/W) User Control (Enable DMP, FIFO, etc.)
#define PWR_MGMT_1       0x6B  // (R/W) Power Management 1
#define PWR_MGMT_2       0x6C  // (R/W) Power Management 2
#define WHO_AM_I_MPU9250 0x75  // (R) Who Am I register, should return 0x71

#endif // MPU9250_REGISTERS_H
