#include <stdio.h>
#include <wiringPi.h>
#include "mpu9250.h"

int main() {
    int fd;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    if (wiringPiSetup() == -1) {
        printf("Error al inicializar wiringPi.\\n");
        return -1;
    }

    fd = mpu9250_init(MPU9250_ADDR);
    if (fd == -1) {
        return -1;
    }

    while (1) {
        mpu9250_read_accel(fd, &ax, &ay, &az);
        mpu9250_read_gyro(fd, &gx, &gy, &gz);

        printf("Acelerómetro: X=%6d, Y=%6d, Z=%6d | ", ax, ay, az);
        printf("Giroscopio: X=%6d, Y=%6d, Z=%6d\\r", gx, gy, gz);
        fflush(stdout);

        delay(100);
    }

    return 0;
}
