/* class for using an MPU6050 IMU sensor on pi pico.
An abstraction of pico example C code. 
TODO:
    smart ptrs for results
    C++ scale enums
    mpu6050_t struct in C code for storing scales and bus addr
 */
#include "pico/stdlib.h"

class MPU6050 {
    float *accel, *gyro, temp;
    uint8_t accel_scale, gyro_scale;
public:
    const float &chip_temp; // [C]

    MPU6050(); //TODO
    MPU6050(float *accel_out, float *gyro_out); // [m/s^2], [rad/s]

    void power(uint8_t CLKSEL, bool temp_disable, bool sleep, bool cycle);
    void reset(void);

    void setscale_accel(uint8_t scale_num); //scale 0-3 is 2g, 4g, 8g, or 16g
    void setscale_gyro(uint8_t scale_num); // scale 0-3 is 250, 500, 1000, or 2000 deg/s
    void read(void);
};
