/* class for using an MPU6050 IMU sensor on pi pico.
An abstraction of pico example C code. */

class MPU6050 {
    float *accel, *gyro, temp;
public:
    const float &accel[3], &gyro[3], &chip_temp; // [m/s^2], [rad/s], [C]
    MPU6050(void);
    MPU6050(float *accel_out, float *gyro_out);
    void power(uint8_t CLKSEL, bool temp_disable, bool sleep, bool cycle);
    void reset(void);
    void setscale_accel(uint8_t scale_num); //scale 0-3 is 2g, 4g, 8g, or 16g
    void setscale_gyro(uint8_t scale_num); // scale 0-3 is 250, 500, 1000, or 2000 deg/s
    void read(void);
}