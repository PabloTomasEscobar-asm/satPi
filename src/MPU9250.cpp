#include "MPU9250.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>
#include <unistd.h>
#include <cmath>

// ---- Public Methods ----

MPU9250::MPU9250() {
    // Constructor is intentionally left empty.
    // Initialization is handled by the init() method.
}

bool MPU9250::init(Ascale ascale, Gscale gscale, Mscale mscale, Mmode mmode) {
    // Store the configuration
    _ascale = ascale;
    _gscale = gscale;
    _mscale = mscale;
    _mmode = mmode;

    // Initialize wiringPi and I2C communication
    wiringPiSetup();
    _mpu_fd = wiringPiI2CSetup(MPU9250_ADDRESS);

    if (_mpu_fd == -1) {
        std::cerr << "ERROR: Failed to initialize I2C for MPU9250." << std::endl;
        return false;
    }

    // Enable I2C Bypass Mode to access the AK8963 magnetometer.
    // This makes the AK8963 visible on the main I2C bus.
    writeByte(_mpu_fd, INT_PIN_CFG, 0x02);
    delay(10); // Wait for the bypass to activate

    _mag_fd = wiringPiI2CSetup(AK8963_ADDRESS);

    if (_mag_fd == -1) {
        std::cerr << "ERROR: Failed to initialize I2C for AK8963." << std::endl;
        return false;
    }

    // Verify sensor connection
    if (!whoAmI()) {
        return false;
    }
    
    // Reset the devices to a known state
    reset();

    // Calculate sensor resolutions based on selected scales
    updateResolutions();

    // Initialize the MPU9250 (Accel & Gyro)
    initMPU9250();
    std::cout << "MPU9250 initialized successfully." << std::endl;

    // Initialize the AK8963 (Magnetometer)
    initAK8963();
    std::cout << "AK8963 initialized successfully." << std::endl;

    return true;
}

bool MPU9250::whoAmI() {
    uint8_t mpu_id = readByte(_mpu_fd, WHO_AM_I_MPU9250);
    bool mpu_ok = (mpu_id == 0x71 || mpu_id == 0x73);
    if (!mpu_ok) {
        std::cerr << "ERROR: MPU9250 WHO_AM_I check failed. Expected 0x71 or 0x73, got 0x" << std::hex << (int)mpu_id << std::dec << std::endl;
    }

    uint8_t mag_id = readByte(_mag_fd, AK8963_WHO_AM_I);
    bool mag_ok = (mag_id == 0x48);
    if (!mag_ok) {
        std::cerr << "ERROR: AK8963 WHO_AM_I check failed. Expected 0x48, got 0x" << std::hex << (int)mag_id << std::dec << std::endl;
    }
    
    return mpu_ok && mag_ok;
}

void MPU9250::reset() {
    // Reset MPU9250
    writeByte(_mpu_fd, PWR_MGMT_1, 0x80);
    delay(100);
    
    // Reset AK8963
    writeByte(_mag_fd, AK8963_CNTL2, 0x01);
    delay(100);
}

void MPU9250::update() {
    int16_t rawData[3];
    
    // Read accelerometer data
    readAccelData(rawData);
    ax = (float)rawData[0] * _aRes - accelBias[0];
    ay = (float)rawData[1] * _aRes - accelBias[1];
    az = (float)rawData[2] * _aRes - accelBias[2];

    // Read gyroscope data
    readGyroData(rawData);
    gx = (float)rawData[0] * _gRes - gyroBias[0];
    gy = (float)rawData[1] * _gRes - gyroBias[1];
    gz = (float)rawData[2] * _gRes - gyroBias[2];

    // Read magnetometer data
    readMagData(rawData);
    mx = (float)rawData[0] * _mRes * magCalibration[0] - magBias[0];
    my = (float)rawData[1] * _mRes * magCalibration[1] - magBias[1];
    mz = (float)rawData[2] * _mRes * magCalibration[2] - magBias[2];
    
    // Read temperature data
    temperature = ((float)readTempData()) / 333.87f + 21.0f;
}

void MPU9250::calibrate() {
    uint8_t data[12];
    uint16_t fifo_count;
    int32_t gyro_bias_sum[3] = {0, 0, 0}, accel_bias_sum[3] = {0, 0, 0};

    std::cout << "Starting calibration. Keep the sensor flat and motionless." << std::endl;
    delay(2000);

    // Reset device
    reset();

    // Configure device for bias calculation
    writeByte(_mpu_fd, PWR_MGMT_1, 0x00);   // Clear sleep
    delay(200);
    writeByte(_mpu_fd, PWR_MGMT_1, 0x01);   // Set clock source to PLL
    writeByte(_mpu_fd, INT_ENABLE, 0x00);   // Disable all interrupts
    writeByte(_mpu_fd, FIFO_EN, 0x00);      // Disable FIFO
    writeByte(_mpu_fd, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    writeByte(_mpu_fd, USER_CTRL, 0x04);    // Reset FIFO
    delay(15);
    
    // Configure MPU9250 for bias calculation
    writeByte(_mpu_fd, CONFIG, 0x01);      // Set DLPF to 184 Hz
    writeByte(_mpu_fd, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    writeByte(_mpu_fd, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 dps
    writeByte(_mpu_fd, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g

    // Configure FIFO to capture accelerometer and gyro data
    writeByte(_mpu_fd, USER_CTRL, 0x40);   // Enable FIFO
    writeByte(_mpu_fd, FIFO_EN, 0x78);     // Enable accel and gyro FIFO
    delay(80); // Accumulate 80 samples in 80 milliseconds

    // Stop FIFO
    writeByte(_mpu_fd, FIFO_EN, 0x00);
    
    // Read FIFO sample count
    readBytes(_mpu_fd, FIFO_COUNTH, 2, &data[0]);
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    int packet_count = fifo_count / 12;

    for (int i = 0; i < packet_count; i++) {
        int16_t accel_temp[3], gyro_temp[3];
        readBytes(_mpu_fd, FIFO_R_W, 12, &data[0]);
        accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);
        accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
        accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
        gyro_temp[0]  = (int16_t)(((int16_t)data[6] << 8) | data[7]);
        gyro_temp[1]  = (int16_t)(((int16_t)data[8] << 8) | data[9]);
        gyro_temp[2]  = (int16_t)(((int16_t)data[10] << 8) | data[11]);
        
        accel_bias_sum[0] += accel_temp[0];
        accel_bias_sum[1] += accel_temp[1];
        accel_bias_sum[2] += accel_temp[2];
        gyro_bias_sum[0]  += gyro_temp[0];
        gyro_bias_sum[1]  += gyro_temp[1];
        gyro_bias_sum[2]  += gyro_temp[2];
    }

    // Average the biases
    if(packet_count > 0) {
        accel_bias_sum[0] /= packet_count;
        accel_bias_sum[1] /= packet_count;
        accel_bias_sum[2] /= packet_count;
        gyro_bias_sum[0]  /= packet_count;
        gyro_bias_sum[1]  /= packet_count;
        gyro_bias_sum[2]  /= packet_count;
    }

    // Remove gravity from Z-axis accelerometer bias
    float accel_sens = 16384.0f; // = 1g for AFS_2G
    if (accel_bias_sum[2] > 0L) {
        accel_bias_sum[2] -= accel_sens;
    } else {
        accel_bias_sum[2] += accel_sens;
    }

    // Store biases in real-world units
    updateResolutions(); // Ensure resolutions are up-to-date
    accelBias[0] = (float)accel_bias_sum[0] * _aRes;
    accelBias[1] = (float)accel_bias_sum[1] * _aRes;
    accelBias[2] = (float)accel_bias_sum[2] * _aRes;
    gyroBias[0] = (float)gyro_bias_sum[0] * _gRes;
    gyroBias[1] = (float)gyro_bias_sum[1] * _gRes;
    gyroBias[2] = (float)gyro_bias_sum[2] * _gRes;
    
    std::cout << "Calibration complete." << std::endl;

    // Restore original configuration
    initMPU9250();
}

void MPU9250::selfTest() {
    std::cout << "Self-test function not yet fully implemented." << std::endl;
}

float MPU9250::getAccelRes() { return _aRes; }
float MPU9250::getGyroRes() { return _gRes; }
float MPU9250::getMagRes() { return _mRes; }

void MPU9250::enableWakeOnMotion(float threshold_mg) {
    writeByte(_mpu_fd, PWR_MGMT_1, 0x01);
    writeByte(_mpu_fd, PWR_MGMT_2, 0x00);
    delay(10);
    writeByte(_mpu_fd, ACCEL_CONFIG2, ACCEL_DLPF_184HZ);
    writeByte(_mpu_fd, INT_ENABLE, 0x40);
    writeByte(_mpu_fd, MOT_DETECT_CTRL, 0xC0);
    uint8_t threshold_lsb = (uint8_t)(threshold_mg / 4.0f);
    if (threshold_lsb < 1) threshold_lsb = 1;
    if (threshold_lsb > 255) threshold_lsb = 255;
    writeByte(_mpu_fd, WOM_THR, threshold_lsb);
    writeByte(_mpu_fd, LP_ACCEL_ODR, 0x06);
    uint8_t pwr_mgmt_1 = readByte(_mpu_fd, PWR_MGMT_1);
    writeByte(_mpu_fd, PWR_MGMT_1, pwr_mgmt_1 | 0x20);
    writeByte(_mpu_fd, INT_PIN_CFG, 0x30);
    std::cout << "Wake-on-Motion enabled with a threshold of " << (threshold_lsb * 4) << " mg." << std::endl;
}

uint8_t MPU9250::getInterruptStatus() {
    return readByte(_mpu_fd, INT_STATUS);
}

// ---- Private Methods ----

void MPU9250::writeByte(int fd, uint8_t reg, uint8_t data) {
    wiringPiI2CWriteReg8(fd, reg, data);
}

uint8_t MPU9250::readByte(int fd, uint8_t reg) {
    return wiringPiI2CReadReg8(fd, reg);
}

void MPU9250::readBytes(int fd, uint8_t reg, uint8_t count, uint8_t* dest) {
    for (int i = 0; i < count; i++) {
        dest[i] = readByte(fd, reg + i);
    }
}

void MPU9250::readAccelData(int16_t* destination) {
    uint8_t rawData[6];
    readBytes(_mpu_fd, ACCEL_XOUT_H, 6, &rawData[0]);
    destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
    destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
    destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
}

void MPU9250::readGyroData(int16_t* destination) {
    uint8_t rawData[6];
    readBytes(_mpu_fd, GYRO_XOUT_H, 6, &rawData[0]);
    destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
    destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
    destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
}

void MPU9250::readMagData(int16_t* destination) {
    uint8_t rawData[7];
    if (readByte(_mag_fd, AK8963_ST1) & 0x01) {
        readBytes(_mag_fd, AK8963_XOUT_L, 7, &rawData[0]);
        if (!(rawData[6] & 0x08)) {
            destination[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);
            destination[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]);
            destination[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]);
        }
    }
}

int16_t MPU9250::readTempData() {
    uint8_t rawData[2];
    readBytes(_mpu_fd, TEMP_OUT_H, 2, &rawData[0]);
    return (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
}

void MPU9250::initMPU9250() {
    writeByte(_mpu_fd, PWR_MGMT_1, 0x01); // Set clock source to auto select

    writeByte(_mpu_fd, CONFIG, GYRO_DLPF_41HZ); 
    writeByte(_mpu_fd, SMPLRT_DIV, 0x04); // Set sample rate to 200Hz (1kHz / (1+4))
    
    uint8_t c = readByte(_mpu_fd, GYRO_CONFIG);
    writeByte(_mpu_fd, GYRO_CONFIG, (c & ~0x18) | (_gscale << 3));
    
    c = readByte(_mpu_fd, ACCEL_CONFIG);
    writeByte(_mpu_fd, ACCEL_CONFIG, (c & ~0x18) | (_ascale << 3));

    c = readByte(_mpu_fd, ACCEL_CONFIG2);
    writeByte(_mpu_fd, ACCEL_CONFIG2, (c & ~0x0F) | ACCEL_DLPF_41HZ);

    writeByte(_mpu_fd, INT_PIN_CFG, 0x22);    
    writeByte(_mpu_fd, INT_ENABLE, 0x01);
    delay(100);
}

void MPU9250::initAK8963() {
    uint8_t rawData[3];
    writeByte(_mag_fd, AK8963_CNTL, M_POWER_DOWN);
    delay(10);
    writeByte(_mag_fd, AK8963_CNTL, M_FUSE_ROM_ACCESS);
    delay(10);
    
    readBytes(_mag_fd, AK8963_ASAX, 3, &rawData[0]);
    magCalibration[0] = (float)(rawData[0] - 128) / 256.0f + 1.0f;
    magCalibration[1] = (float)(rawData[1] - 128) / 256.0f + 1.0f;
    magCalibration[2] = (float)(rawData[2] - 128) / 256.0f + 1.0f;
    
    writeByte(_mag_fd, AK8963_CNTL, M_POWER_DOWN);
    delay(10);
    
    writeByte(_mag_fd, AK8963_CNTL, (_mscale << 4) | _mmode);
    delay(10);
}

void MPU9250::updateResolutions() {
    switch (_ascale) {
        case AFS_2G:  _aRes = 2.0f / 32768.0f; break;
        case AFS_4G:  _aRes = 4.0f / 32768.0f; break;
        case AFS_8G:  _aRes = 8.0f / 32768.0f; break;
        case AFS_16G: _aRes = 16.0f / 32768.0f; break;
    }
    switch (_gscale) {
        case GFS_250DPS:  _gRes = 250.0f / 32768.0f; break;
        case GFS_500DPS:  _gRes = 500.0f / 32768.0f; break;
        case GFS_1000DPS: _gRes = 1000.0f / 32768.0f; break;
        case GFS_2000DPS: _gRes = 2000.0f / 32768.0f; break;
    }
    switch (_mscale) {
        case MFS_14BITS: _mRes = 10.0f * 4912.0f / 8190.0f; break;
        case MFS_16BITS: _mRes = 10.0f * 4912.0f / 32760.0f; break;
    }
}
