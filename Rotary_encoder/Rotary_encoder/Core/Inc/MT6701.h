#ifndef __MT6701_H__
#define __MT6701_H__

#include "stm32f1xx_hal.h"
#include "i2c.h"
#include <stdint.h>
#include <stdio.h>

/*********************************************************************
 * MT6701 DEFINITIONS
 */
#define MT6701_SLAVE_ADDR           (0x06 << 1)
#define MT6701_TIMEOUT              50

#define MT6701_REG_ANGLE_14b        0x03 // 14-bit angle information

/*********************************************************************
 * TCA9548A DEFINITIONS
 */
#define TCA9548A_SLAVE_ADDR         0x70
#define TCA9548A_TIMEOUT            50

#define TCA9548A_CHANNEL_0          0x01
#define TCA9548A_CHANNEL_1          0x02
#define TCA9548A_CHANNEL_2          0x04
#define TCA9548A_CHANNEL_3          0x08
#define TCA9548A_CHANNEL_4          0x10
#define TCA9548A_CHANNEL_5          0x20
#define TCA9548A_CHANNEL_6          0x40
#define TCA9548A_CHANNEL_7          0x80

/*********************************************************************
 * API FUNCTIONS
 */

// TCA9548A Functions
void tca9548a_set_channel(I2C_HandleTypeDef *hi2c, uint8_t channel);

// MT6701 Functions (direct connection)
unsigned char mt6701_write_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value);
unsigned char mt6701_read_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t* buf, uint16_t len);
void i2c_mt6701_get_angle(I2C_HandleTypeDef *hi2c, int16_t *angle_int, float *angle);

// MT6701 Functions (via TCA9548A)
unsigned char mt6701_write_reg_with_mux(I2C_HandleTypeDef *hi2c, uint8_t channel, uint8_t reg, uint8_t value);
unsigned char mt6701_read_reg_with_mux(I2C_HandleTypeDef *hi2c, uint8_t channel, uint8_t reg, uint8_t* buf, uint16_t len);
void i2c_mt6701_get_angle_with_mux(I2C_HandleTypeDef *hi2c, uint8_t channel, int16_t *angle_int, float *angle);

#endif /* __MT6701_H__ */
