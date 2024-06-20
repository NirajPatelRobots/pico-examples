#include "mpu6050.hpp"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "mpu6050_i2c.h"

/*MPU6050::MPU6050() :  //TODO: smart pointers
chip_temp(*temp) {
    accel = {0., 0., 0.};
    gyro = {0., 0., 0.};
    temp = {};
    MPU6050(accel, gyro, temp);
} */

MPU6050::MPU6050(float *accel_out, float *gyro_out) :
 accel(accel_out),  gyro(gyro_out), chip_temp(temp) {
    accel_scale = 0;
    gyro_scale = 0;
    setup_MPU6050_i2c();
}

void MPU6050::power(uint8_t CLKSEL, bool temp_disable, bool sleep, bool cycle) {
    mpu6050_power(CLKSEL, temp_disable, sleep, cycle);
}

void MPU6050::reset(void) {
    mpu6050_reset();
}

void MPU6050::setscale_accel(uint8_t scale_num) {
    accel_scale = scale_num & 3;
    mpu6050_setscale_accel((MPU6050_Scale)accel_scale);
}

void MPU6050::setscale_gyro(uint8_t scale_num) {
    gyro_scale = scale_num & 3;
    mpu6050_setscale_gyro((MPU6050_Scale)gyro_scale);
}

void MPU6050::read(void) {
    mpu6050_read(accel, gyro, &temp, (MPU6050_Scale)accel_scale, (MPU6050_Scale)gyro_scale);
}
    