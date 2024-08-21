/* class for using an MPU6050 IMU sensor on pi pico.
An abstraction of pico example C code. 
TODO:
    smart ptrs for results
 */
#include "stdint.h"

struct MPU6050SensorTimingParams {
    MPU6050SensorTimingParams() = default;
    MPU6050SensorTimingParams(int _bandwidth, float _delay, float _sample_rate);
    int bandwidth; // lowpass filter bandwidth [Hz]
    float delay; // lowpass filter delay [ms]
    float sample_rate; // rate of new data loading in the register [Hz]
};
struct MPU6050TimingParams {
    MPU6050TimingParams(uint8_t lowpass_filter_cfg, uint8_t sample_rate_div);
    MPU6050TimingParams(const MPU6050SensorTimingParams &_accel_timing, const MPU6050SensorTimingParams &_gyro_timing);
    MPU6050SensorTimingParams accel_timing, gyro_timing;
};

class MPU6050 {
public:
    const float &chip_temp; // [C]

    MPU6050(); //TODO
    MPU6050(float *accel_out, float *gyro_out, uint8_t i2c_addr=0x68); // [m/s^2], [rad/s]

    void power(uint8_t CLKSEL, bool temp_disable, bool sleep, bool cycle);
    void reset(void);

    enum Scale {Scale_0 = 0, Scale_1, Scale_2, Scale_3};
    // Warning: The first call to read() after setscale() might not have the updated scaling.
    void setscale_accel(Scale scale); //scale 0-3 is 2g, 4g, 8g, or 16g
    void setscale_gyro(Scale scale); // scale 0-3 is 250, 500, 1000, or 2000 deg/s
    void read(void);
    bool is_connected(void);
    void set_timing(uint8_t lowpass_filter_cfg, uint8_t sample_rate_div);
    MPU6050TimingParams read_timing(void);
private:
    float *accel, *gyro, temp;
    enum Scale accel_scale, gyro_scale;
    uint8_t bus_addr;
};
