#ifndef __LIS3MDL_DRIVER_H
#define __LIS3MDL_DRIVER_H
#include <mbed.h>

class LIS3MDLDriver {
public:
    LIS3MDLDriver(I2C *driver, int32_t addr = 0x3D)
    {
        this->i2c_driver = driver;
        this->dev_addr = addr;
    }
    int32_t init();
    int32_t deinit();
    int32_t read_id(uint8_t *id);
    int32_t set_low_power(bool enable);
    int32_t read_xyz(int16_t *x, int16_t *y, int16_t *z);

private:
    I2C *i2c_driver;
    int32_t dev_addr;
};

#endif /* __LIS3MDL_DRIVER_H */