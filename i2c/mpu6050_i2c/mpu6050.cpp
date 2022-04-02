#include "mpu6050.hpp"
#include "mpu6050_i2c.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

static MPU6050_Scale MPU6050::accel_scale;
static MPU6050_Scale MPU6050::gyro_scale;

/*MPU6050::MPU6050() :  //TODO: is there a good way to do this
chip_temp(*temp) {
    accel = {0., 0., 0.};
    gyro = {0., 0., 0.};
    temp = {};
    MPU6050(accel, gyro, temp);
} */

MPU6050::MPU6050(float *accel_out, float *gyro_out) :
 accel(accel_out),  gyro(gyro_out), chip_temp(temp) {
    accel_scale = MPU_FS_0;
    gyro_scale = MPU_FS_0;
    //I2C init
    i2c_init(i2c_default, 400 * 1000); // Max bus speed 400 kHz
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
}

MPU6050::power(uint8_t CLKSEL, bool temp_disable, bool sleep, bool cycle) {
    mpu6050_power(CLKSEL, temp_disable, sleep, cycle);
}

MPU6050::reset(void) {
    mpu6050_reset();
}

MPU6050::setscale_accel(uint8_t scale_num) {
    accel_scale = (MPU6050_Scale)(scale_num & 3);
    mpu6050_setscale_accel(accel_scale);
}

MPU6050::setscale_gyro(uint8_t scale_num) {
    gyro_scale = (MPU6050_Scale)(scale_num & 3);
    mpu6050_setscale_gyro(gyro_scale);
}

MPU6050::read(void) {
    mpu6050_read(accel, gyro, &temp, accel_scale, gyro_scale);
}
    