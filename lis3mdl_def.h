#ifndef __LIS3MDL_DEF_H
#define __LIS3MDL_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Registers */
#define LIS3MDL_REGISTER_WHO_AM_I       (uint8_t)0x0F // Device identification
#define LIS3MDL_REGISTER_CTRL_REG1      (uint8_t)0x20
#define LIS3MDL_REGISTER_CTRL_REG2      (uint8_t)0x21
#define LIS3MDL_REGISTER_CTRL_REG3      (uint8_t)0x22
#define LIS3MDL_REGISTER_CTRL_REG4      (uint8_t)0x23
#define LIS3MDL_REGISTER_CTRL_REG5      (uint8_t)0x24
#define LIS3MDL_REGISTER_STATUS_REG     (uint8_t)0x27
#define LIS3MDL_REGISTER_OUT_X_L        (uint8_t)0x28
#define LIS3MDL_REGISTER_OUT_X_H        (uint8_t)0x29
#define LIS3MDL_REGISTER_OUT_Y_L        (uint8_t)0x2A
#define LIS3MDL_REGISTER_OUT_Y_H        (uint8_t)0x2B
#define LIS3MDL_REGISTER_OUT_Z_L        (uint8_t)0x2C
#define LIS3MDL_REGISTER_OUT_Z_H        (uint8_t)0x2D
#define LIS3MDL_REGISTER_TEMP_OUT_L     (uint8_t)0x2E
#define LIS3MDL_REGISTER_TEMP_OUT_H     (uint8_t)0x2F
#define LIS3MDL_REGISTER_INT_CFG        (uint8_t)0x30
#define LIS3MDL_REGISTER_INT_SRC        (uint8_t)0x31
#define LIS3MDL_REGISTER_INT_THS_L      (uint8_t)0x32
#define LIS3MDL_REGISTER_INT_THS_H      (uint8_t)0x33

/* WHO_AM_I Register */
#define LIS3MDL_WHO_AM_I_VALUE           (uint8_t)0x3D

/* Mag Temperature Sensor Control*/ 
#define LIS3MDL_MAG_TEMPSENSOR_ENABLE        ((uint8_t) 0x80)   /*!< Temp sensor Enable */
#define LIS3MDL_MAG_TEMPSENSOR_DISABLE       ((uint8_t) 0x00)   /*!< Temp sensor Disable */

/* Mag_XY-axis Operating Mode */ 
#define LIS3MDL_MAG_OM_XY_LOWPOWER           ((uint8_t) 0x00)
#define LIS3MDL_MAG_OM_XY_MEDIUM             ((uint8_t) 0x20)
#define LIS3MDL_MAG_OM_XY_HIGH               ((uint8_t) 0x40)
#define LIS3MDL_MAG_OM_XY_ULTRAHIGH          ((uint8_t) 0x60)
   
/* Mag Data Rate */ 
#define LIS3MDL_MAG_ODR_0_625_HZ             ((uint8_t) 0x00)  /*!< Output Data Rate = 0.625 Hz */
#define LIS3MDL_MAG_ODR_1_25_HZ              ((uint8_t) 0x04)  /*!< Output Data Rate = 1.25 Hz  */
#define LIS3MDL_MAG_ODR_2_5_HZ               ((uint8_t) 0x08)  /*!< Output Data Rate = 2.5 Hz   */
#define LIS3MDL_MAG_ODR_5_0_HZ               ((uint8_t) 0x0C)  /*!< Output Data Rate = 5.0 Hz   */
#define LIS3MDL_MAG_ODR_10_HZ                ((uint8_t) 0x10)  /*!< Output Data Rate = 10 Hz    */
#define LIS3MDL_MAG_ODR_20_HZ                ((uint8_t) 0x14)  /*!< Output Data Rate = 20 Hz    */
#define LIS3MDL_MAG_ODR_40_HZ                ((uint8_t) 0x18)  /*!< Output Data Rate = 40 Hz    */
#define LIS3MDL_MAG_ODR_80_HZ                ((uint8_t) 0x1C)  /*!< Output Data Rate = 80 Hz    */

/* Mag Data Rate */ 
#define LMS303C_MAG_SELFTEST_DISABLE         ((uint8_t 0x00)     
#define LMS303C_MAG_SELFTEST_ENABLE          ((uint8_t 0x01)
   
/* Mag Full Scale */ 
#define LIS3MDL_MAG_FS_DEFAULT               ((uint8_t) 0x00)
#define LIS3MDL_MAG_FS_4_GA                  ((uint8_t) 0x00)  
#define LIS3MDL_MAG_FS_8_GA                  ((uint8_t) 0x20)
#define LIS3MDL_MAG_FS_12_GA                 ((uint8_t) 0x40)  
#define LIS3MDL_MAG_FS_16_GA                 ((uint8_t) 0x60)  /*!< Full scale = �16 Gauss */

/* Mag_Reboot */ 
#define LIS3MDL_MAG_REBOOT_DEFAULT           ((uint8_t) 0x00)
#define LIS3MDL_MAG_REBOOT_ENABLE            ((uint8_t) 0x08)
   
/* Mag Soft reset */ 
#define LIS3MDL_MAG_SOFT_RESET_DEFAULT       ((uint8_t) 0x00)
#define LIS3MDL_MAG_SOFT_RESET_ENABLE        ((uint8_t) 0x04)
   
/* Mag_Communication_Mode */ 
#define LIS3MDL_MAG_SIM_4_WIRE               ((uint8_t) 0x00)
#define LIS3MDL_MAG_SIM_3_WIRE               ((uint8_t) 0x04)
   
/* Mag Lowpower mode config */ 
#define LIS3MDL_MAG_CONFIG_NORMAL_MODE       ((uint8_t) 0x00)
#define LIS3MDL_MAG_CONFIG_LOWPOWER_MODE     ((uint8_t) 0x20)
   
/* Mag Operation Mode */ 
#define LIS3MDL_MAG_SELECTION_MODE           ((uint8_t) 0x03) /* CTRL_REG3 */
#define LIS3MDL_MAG_CONTINUOUS_MODE          ((uint8_t) 0x00)
#define LIS3MDL_MAG_SINGLE_MODE              ((uint8_t) 0x01)
#define LIS3MDL_MAG_POWERDOWN1_MODE          ((uint8_t) 0x02)
#define LIS3MDL_MAG_POWERDOWN2_MODE          ((uint8_t) 0x03)

/* Mag_Z-axis Operation Mode */ 
#define LIS3MDL_MAG_OM_Z_LOWPOWER            ((uint8_t) 0x00)
#define LIS3MDL_MAG_OM_Z_MEDIUM              ((uint8_t) 0x04)
#define LIS3MDL_MAG_OM_Z_HIGH                ((uint8_t) 0x08)
#define LIS3MDL_MAG_OM_Z_ULTRAHIGH           ((uint8_t) 0x0C)   

/* Mag Big little-endian selection */ 
#define LIS3MDL_MAG_BLE_LSB                  ((uint8_t) 0x00)
#define LIS3MDL_MAG_BLE_MSB                  ((uint8_t) 0x02)


/* Mag_Bloc_update_magnetic_data */ 
#define LIS3MDL_MAG_BDU_CONTINUOUS           ((uint8_t) 0x00)
#define LIS3MDL_MAG_BDU_MSBLSB               ((uint8_t) 0x40)
   
   
/* Magnetometer_Sensitivity */
#define LIS3MDL_MAG_SENSITIVITY_FOR_FS_4GA   ((float)0.14f)  /**< Sensitivity value for 4 gauss full scale  [mgauss/LSB] */
#define LIS3MDL_MAG_SENSITIVITY_FOR_FS_8GA   ((float)0.29f)  /**< Sensitivity value for 8 gauss full scale  [mgauss/LSB] */
#define LIS3MDL_MAG_SENSITIVITY_FOR_FS_12GA  ((float)0.43f)  /**< Sensitivity value for 12 gauss full scale [mgauss/LSB] */
#define LIS3MDL_MAG_SENSITIVITY_FOR_FS_16GA  ((float)0.58f)  /**< Sensitivity value for 16 gauss full scale [mgauss/LSB] */


#ifdef __cplusplus
}
#endif

#endif /* __LIS3MDL_DEF_H */
