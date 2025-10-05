#ifndef MPU9250_REGISTERS_H
#define MPU9250_REGISTERS_H

// ===============================================================================
//                              Device I2C Addresses
// ===============================================================================
#define MPU9250_ADDRESS   0x68   // Device address when ADO = 0
#define AK8963_ADDRESS    0x0C   // Address of magnetometer

// ===============================================================================
//                              AK8963 Magnetometer Registers
// ===============================================================================
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
#define AK8963_CNTL      0x0A   // (R/W) Power down, single-measurement, self-test, fuse ROM modes
#define AK8963_CNTL2     0x0B   // (R/W) Reset
#define AK8963_ASTC      0x0C   // (R/W) Self test control
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

// ===============================================================================
//                              MPU9250 Gyro & Accel Offset Registers
// ===============================================================================
#define XG_OFFSET_H       0x13
#define XG_OFFSET_L       0x14
#define YG_OFFSET_H       0x15
#define YG_OFFSET_L       0x16
#define ZG_OFFSET_H       0x17
#define ZG_OFFSET_L       0x18
#define XA_OFFSET_H       0x77
#define XA_OFFSET_L       0x78
#define YA_OFFSET_H       0x7A
#define YA_OFFSET_L       0x7B
#define ZA_OFFSET_H       0x7D
#define ZA_OFFSET_L       0x7E

// ===============================================================================
//                              MPU9250 Configuration Registers
// ===============================================================================
#define SMPLRT_DIV        0x19
#define CONFIG            0x1A
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define ACCEL_CONFIG2     0x1D
#define LP_ACCEL_ODR      0x1E
#define WOM_THR           0x1F

// ===============================================================================
//                              MPU9250 FIFO Registers
// ===============================================================================
#define FIFO_EN           0x23
#define FIFO_COUNTH       0x72
#define FIFO_COUNTL       0x73
#define FIFO_R_W          0x74

// ===============================================================================
//                              MPU9250 I2C Master Control Registers
// ===============================================================================
#define I2C_MST_CTRL      0x24
#define I2C_SLV0_ADDR     0x25
#define I2C_SLV0_REG      0x26
#define I2C_SLV0_CTRL     0x27
#define I2C_SLV1_ADDR     0x28
#define I2C_SLV1_REG      0x29
#define I2C_SLV1_CTRL     0x2A
#define I2C_SLV2_ADDR     0x2B
#define I2C_SLV2_REG      0x2C
#define I2C_SLV2_CTRL     0x2D
#define I2C_SLV3_ADDR     0x2E
#define I2C_SLV3_REG      0x2F
#define I2C_SLV3_CTRL     0x30
#define I2C_SLV4_ADDR     0x31
#define I2C_SLV4_REG      0x32
#define I2C_SLV4_DO       0x33
#define I2C_SLV4_CTRL     0x34
#define I2C_SLV4_DI       0x35
#define I2C_MST_STATUS    0x36
#define I2C_SLV0_DO       0x63
#define I2C_SLV1_DO       0x64
#define I2C_SLV2_DO       0x65
#define I2C_SLV3_DO       0x66
#define I2C_MST_DELAY_CTRL 0x67

// ===============================================================================
//                              MPU9250 Interrupt Registers
// ===============================================================================
#define INT_PIN_CFG       0x37
#define INT_ENABLE        0x38
#define INT_STATUS        0x3A

// ===============================================================================
//                              MPU9250 Sensor Data Output Registers
// ===============================================================================
#define ACCEL_XOUT_H      0x3B
#define ACCEL_XOUT_L      0x3C
#define ACCEL_YOUT_H      0x3D
#define ACCEL_YOUT_L      0x3E
#define ACCEL_ZOUT_H      0x3F
#define ACCEL_ZOUT_L      0x40
#define TEMP_OUT_H        0x41
#define TEMP_OUT_L        0x42
#define GYRO_XOUT_H       0x43
#define GYRO_XOUT_L       0x44
#define GYRO_YOUT_H       0x45
#define GYRO_YOUT_L       0x46
#define GYRO_ZOUT_H       0x47
#define GYRO_ZOUT_L       0x48

// ===============================================================================
//                              MPU9250 External Sensor Data Registers
// ===============================================================================
#define EXT_SENS_DATA_00  0x49
#define EXT_SENS_DATA_01  0x4A
#define EXT_SENS_DATA_02  0x4B
#define EXT_SENS_DATA_03  0x4C
#define EXT_SENS_DATA_04  0x4D
#define EXT_SENS_DATA_05  0x4E
#define EXT_SENS_DATA_06  0x4F
#define EXT_SENS_DATA_07  0x50
#define EXT_SENS_DATA_08  0x51
#define EXT_SENS_DATA_09  0x52
#define EXT_SENS_DATA_10  0x53
#define EXT_SENS_DATA_11  0x54
#define EXT_SENS_DATA_12  0x55
#define EXT_SENS_DATA_13  0x56
#define EXT_SENS_DATA_14  0x57
#define EXT_SENS_DATA_15  0x58
#define EXT_SENS_DATA_16  0x59
#define EXT_SENS_DATA_17  0x5A
#define EXT_SENS_DATA_18  0x5B
#define EXT_SENS_DATA_19  0x5C
#define EXT_SENS_DATA_20  0x5D
#define EXT_SENS_DATA_21  0x5E
#define EXT_SENS_DATA_22  0x5F
#define EXT_SENS_DATA_23  0x60

// ===============================================================================
//                              MPU9250 Motion Detection Registers
// ===============================================================================
#define MOT_DETECT_CTRL   0x69
#define MOT_DETECT_STATUS 0x61

// ===============================================================================
//                              MPU9250 Control and Power Registers
// ===============================================================================
#define SIGNAL_PATH_RESET 0x68
#define USER_CTRL         0x6A
#define PWR_MGMT_1        0x6B
#define PWR_MGMT_2        0x6C
#define WHO_AM_I_MPU9250  0x75 // Should return 0x71 or 0x73

#endif // MPU9250_REGISTERS_H


