/**
 * mpu6050 I2c C++ example demonstrating the mpu6050's scaling feature
 */

#include <stdio.h>
#include "mpu6050.hpp"
#include "pico/stdlib.h"

int main() {
    float accel[3], gyro[3]; // TODO we shouldn't need this
    MPU6050 *IMU = new MPU6050(accel, gyro);
    printf("MPU6050 Scaling test\n");
    IMU->reset();
    IMU->power(1, false, false, false);
    printf("Scaled Accelerometer and Gyroscope readings\n");
    printf("Should be roughly equal before and after scale changes\n");
    int accel_scale = 0, gyro_scale = 2;
    while (true) {
        IMU->setscale_accel((MPU6050::Scale)(accel_scale));
        IMU->setscale_gyro((MPU6050::Scale)(gyro_scale));
        printf("\nAcclerometer scale: %i, Gyroscope scale: %i\n", accel_scale, gyro_scale);
        for (int i = 0; i < 100; i++) {
            IMU->read();
            printf("%+6f %+6f %+6f | %+6f %+6f %+6f\r",
                   accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);
            if (i == 0) { printf("\n"); }
            sleep_ms(10);
        }
        accel_scale = (accel_scale + 1) % 4;
        gyro_scale = (gyro_scale + 1) % 4;
    }
}
