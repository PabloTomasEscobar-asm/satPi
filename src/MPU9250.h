#ifndef MPU9250_H
#define MPU9250_H

#include "MPU9250_registers.h"
#include <stdint.h>

// Enums for clear and safe configuration

// Accelerometer Full-Scale Range (Register 28: ACCEL_CONFIG)
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

// Gyroscope Full-Scale Range (Register 27: GYRO_CONFIG)
enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

// Magnetometer Output Resolution (Register 0x0A: CNTL1)
enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

// Magnetometer Operating Mode (Register 0x0A: CNTL1)
enum Mmode {
    M_POWER_DOWN      = 0x00,
    M_SINGLE_MEASURE  = 0x01,
    M_8Hz_CONTINUOUS  = 0x02,
    M_EXT_TRIG_MEASURE= 0x04,
    M_100Hz_CONTINUOUS= 0x06,
    M_SELF_TEST       = 0x08,
    M_FUSE_ROM_ACCESS = 0x0F
};

// Gyroscope Digital Low-Pass Filter (Register 26: CONFIG)
enum GyroDLPF {
    GYRO_DLPF_250HZ = 0,
    GYRO_DLPF_184HZ,
    GYRO_DLPF_92HZ,
    GYRO_DLPF_41HZ,
    GYRO_DLPF_20HZ,
    GYRO_DLPF_10HZ,
    GYRO_DLPF_5HZ,
    GYRO_DLPF_3600HZ
};

// Accelerometer Digital Low-Pass Filter (Register 29: ACCEL_CONFIG 2)
enum AccelDLPF {
    ACCEL_DLPF_460HZ = 0,
    ACCEL_DLPF_184HZ,
    ACCEL_DLPF_92HZ,
    ACCEL_DLPF_41HZ,
    ACCEL_DLPF_20HZ,
    ACCEL_DLPF_10HZ,
    ACCEL_DLPF_5HZ,
    ACCEL_DLPF_460HZ_2 // Same as 0
};

// Clock Source (Register 107: PWR_MGMT_1)
enum ClockSource {
    CLK_INTERNAL_20MHZ = 0,
    CLK_AUTO_SELECT,
    CLK_STOP_CLOCK = 7
};


class MPU9250 {
public:
    // ---- Public Variables ----
    float ax, ay, az, gx, gy, gz, mx, my, mz; // Variables to store sensor data in real-world units (g's, dps, mG)
    float temperature; // Temperature in degrees Celsius

    float gyroBias[3] = {0, 0, 0};
    float accelBias[3] = {0, 0, 0};
    float magBias[3] = {0, 0, 0};
    float magCalibration[3] = {0, 0, 0};


    // ---- Public Methods ----
    MPU9250(); 

    bool init(Ascale ascale = AFS_2G, Gscale gscale = GFS_250DPS, Mscale mscale = MFS_16BITS, Mmode mmode = M_100Hz_CONTINUOUS);
    bool whoAmI();
    void reset();
    void update(); // Reads all sensors and updates public variables

    void calibrate(); // Performs accelerometer and gyroscope calibration
    void selfTest(); // Performs a factory self-test

    // Methods to get current resolution
    float getAccelRes();
    float getGyroRes();
    float getMagRes();

private:
    // ---- Private Member Variables ----
    int _mpu_fd = -1;
    int _mag_fd = -1;
    
    Ascale _ascale;
    Gscale _gscale;
    Mscale _mscale;
    Mmode  _mmode;

    float _aRes, _gRes, _mRes; // Sensor resolutions
    
    // ---- Low-level Private Methods ----
    void writeByte(int fd, uint8_t reg, uint8_t data);
    uint8_t readByte(int fd, uint8_t reg);
    void readBytes(int fd, uint8_t reg, uint8_t count, uint8_t* dest);

    // Raw data reading methods
    void readAccelData(int16_t* destination);
    void readGyroData(int16_t* destination);
    void readMagData(int16_t* destination);
    int16_t readTempData();

    // Internal initialization methods
    void initMPU9250();
    void initAK8963();
    void updateResolutions();
};

#endif // MPU9250_H




