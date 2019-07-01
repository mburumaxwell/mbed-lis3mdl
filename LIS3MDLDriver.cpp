#include "lis3mdl_def.h"
#include "LIS3MDLDriver.h"
#include "mbed_debug.h"

#define DEBUG_ERR_WRITE(a1, a2)  debug_if(a1, "LIS3MDL write 0x%02X failed with result %d", a2, a1)
#define DEBUG_ERR_READ(a1, a2)   debug_if(a1, "LIS3MDL read 0x%02X failed with result %d", a2, a1)

int32_t LIS3MDLDriver::init()
{
    int ret = 0;
    char ctrl1, ctrl2, ctrl3, ctrl4, ctrl5;
    char buf[2];

    ctrl1 = LIS3MDL_MAG_TEMPSENSOR_DISABLE | LIS3MDL_MAG_OM_XY_HIGH | LIS3MDL_MAG_ODR_40_HZ;
    ctrl2 = LIS3MDL_MAG_FS_4_GA | LIS3MDL_MAG_REBOOT_DEFAULT | LIS3MDL_MAG_SOFT_RESET_DEFAULT;
    ctrl3 = LIS3MDL_MAG_CONFIG_NORMAL_MODE | LIS3MDL_MAG_CONTINUOUS_MODE;
    ctrl4 = LIS3MDL_MAG_OM_Z_HIGH | LIS3MDL_MAG_BLE_LSB;
    ctrl5 = LIS3MDL_MAG_BDU_MSBLSB;

    buf[0] = LIS3MDL_REGISTER_CTRL_REG1;
    buf[1] = ctrl1;
    ret = i2c_driver->write(dev_addr, buf, 2);
    DEBUG_ERR_WRITE(ret, buf[0]);
    if (ret) return ret;

    buf[0] = LIS3MDL_REGISTER_CTRL_REG2;
    buf[1] = ctrl2;
    ret = i2c_driver->write(dev_addr, buf, 2);
    DEBUG_ERR_WRITE(ret, buf[0]);
    if (ret) return ret;

    buf[0] = LIS3MDL_REGISTER_CTRL_REG3;
    buf[1] = ctrl3;
    ret = i2c_driver->write(dev_addr, buf, 2);
    DEBUG_ERR_WRITE(ret, buf[0]);
    if (ret) return ret;

    buf[0] = LIS3MDL_REGISTER_CTRL_REG4;
    buf[1] = ctrl4;
    ret = i2c_driver->write(dev_addr, buf, 2);
    DEBUG_ERR_WRITE(ret, buf[0]);
    if (ret) return ret;

    buf[0] = LIS3MDL_REGISTER_CTRL_REG5;
    buf[1] = ctrl5;
    ret = i2c_driver->write(dev_addr, buf, 2);
    DEBUG_ERR_WRITE(ret, buf[0]);
    if (ret) return ret;

    return ret;
}

int32_t LIS3MDLDriver::deinit()
{
    int ret = 0;
    char reg, val;
    
    /* Read CTRL_REG3 */
    reg = LIS3MDL_REGISTER_CTRL_REG3;
    ret = i2c_driver->write(dev_addr, &reg, 1);
    DEBUG_ERR_WRITE(ret, reg);
    if (ret) return ret;
    ret = i2c_driver->read(dev_addr, &val, 1);
    DEBUG_ERR_READ(ret, reg);
    if (ret) return ret;

    /* Clear Selection Mode bits */
    val &= ~(LIS3MDL_MAG_SELECTION_MODE);

    /* Set Power down */
    val |= LIS3MDL_MAG_POWERDOWN2_MODE;
    
    /* Apply settings to CTRL_REG3 */
    reg = LIS3MDL_REGISTER_CTRL_REG3;
    char buf[2] = { reg, val };
    ret = i2c_driver->write(dev_addr, buf, 2);
    DEBUG_ERR_WRITE(ret, reg);
    return ret;
}

int32_t LIS3MDLDriver::read_id(uint8_t *id)
{  
    int ret = 0;
    char reg, val;
  
    /* Read WHO_AM_I */
    reg = LIS3MDL_REGISTER_WHO_AM_I;
    ret = i2c_driver->write(dev_addr, &reg, 1);
    DEBUG_ERR_WRITE(ret, reg);
    if (ret) return ret;
    ret = i2c_driver->read(dev_addr, &val, 1);
    DEBUG_ERR_READ(ret, reg);
    if (ret == 0) *id = (uint8_t)val;
    return ret;
}

int32_t LIS3MDLDriver::set_low_power(bool enable)
{
    int ret = 0;
    char reg, val;
    
    /* Read CTRL_REG3 */
    reg = LIS3MDL_REGISTER_CTRL_REG3;
    ret = i2c_driver->write(dev_addr, &reg, 1);
    DEBUG_ERR_WRITE(ret, reg);
    if (ret) return ret;
    ret = i2c_driver->read(dev_addr, &val, 1);
    DEBUG_ERR_READ(ret, reg);
    if (ret) return ret;

    /* Clear Low Power Mode bit */
    val &= ~(0x20);

    /* Set Low Power Mode */
    if (enable) val |= LIS3MDL_MAG_CONFIG_LOWPOWER_MODE;
    else        val |= LIS3MDL_MAG_CONFIG_NORMAL_MODE;
    
    /* Apply settings to CTRL_REG3 */
    reg = LIS3MDL_REGISTER_CTRL_REG3;
    char buf[2] = { reg, val };
    ret = i2c_driver->write(dev_addr, buf, 2);
    DEBUG_ERR_WRITE(ret, reg);
    return ret;
}

int32_t LIS3MDLDriver::read_xyz(int16_t *x, int16_t *y, int16_t *z)
{
    int ret = 0;
    char reg, ctrlm = 0, buf[6];
    uint8_t i;
    int16_t raw[3];
    float sensitivity = 0;
    
    /* Read CTRL_REG2 */
    reg = LIS3MDL_REGISTER_CTRL_REG2;
    ret = i2c_driver->write(dev_addr, &reg, 1);
    DEBUG_ERR_WRITE(ret, reg);
    if (ret) return ret;
    ret = i2c_driver->read(dev_addr, &ctrlm, 1);
    DEBUG_ERR_READ(ret, reg);
    if (ret) return ret;
    
    /* Read output register X, Y & Z acceleration */
    reg = LIS3MDL_REGISTER_OUT_X_L | 0x80;
    ret = i2c_driver->write(dev_addr, &reg, 1);
    DEBUG_ERR_WRITE(ret, reg);
    if (ret) return ret;
    ret = i2c_driver->read(dev_addr, buf, 6);
    DEBUG_ERR_READ(ret, reg);
    if (ret) return ret;
    
    for (i = 0; i < 3; i++)
    {
        raw[i] = ((((uint16_t)buf[2 * i + 1]) << 8) + (uint16_t)buf[2 * i]);
    }
    
    /* Normal mode */
    /* Switch the sensitivity value set in the CRTL_REG2 */
    switch (ctrlm & 0x60)
    {
        case LIS3MDL_MAG_FS_4_GA:
            sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_FS_4GA;
            break;
        case LIS3MDL_MAG_FS_8_GA:
            sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_FS_8GA;
            break;
        case LIS3MDL_MAG_FS_12_GA:
            sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_FS_12GA;
            break;
        case LIS3MDL_MAG_FS_16_GA:
            sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_FS_16GA;
            break;    
    }
    
    /* Obtain the mGauss value for the three axis */
    *x = (int16_t)(raw[0] * sensitivity);
    *y = (int16_t)(raw[1] * sensitivity);
    *z = (int16_t)(raw[2] * sensitivity);

    return ret;
}