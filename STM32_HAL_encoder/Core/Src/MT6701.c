#include "mt6701.h"

/*********************************************************************
 * TCA9548A FUNCTIONS
 */
void tca9548a_set_channel(I2C_HandleTypeDef *hi2c, uint8_t channel)
{
    uint8_t data = 0;

    switch (channel)
    {
        case 0: data = TCA9548A_CHANNEL_0; break;
        case 1: data = TCA9548A_CHANNEL_1; break;
        case 2: data = TCA9548A_CHANNEL_2; break;
        case 3: data = TCA9548A_CHANNEL_3; break;
        case 4: data = TCA9548A_CHANNEL_4; break;
        case 5: data = TCA9548A_CHANNEL_5; break;
        case 6: data = TCA9548A_CHANNEL_6; break;
        case 7: data = TCA9548A_CHANNEL_7; break;
        default: return; // Invalid channel
    }

    HAL_I2C_Master_Transmit(hi2c, TCA9548A_SLAVE_ADDR << 1, &data, 1, TCA9548A_TIMEOUT);
}

/*********************************************************************
 * MT6701 FUNCTIONS (DIRECT CONNECTION)
 */

// Write a single register
unsigned char mt6701_write_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(hi2c, MT6701_SLAVE_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, MT6701_TIMEOUT);
}

// Read registers
unsigned char mt6701_read_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t* buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(hi2c, MT6701_SLAVE_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, MT6701_TIMEOUT);
}

// Get 14-bit angle
void i2c_mt6701_get_angle(I2C_HandleTypeDef *hi2c, int16_t *angle_int, float *angle)
{
    uint8_t temp[2];
    mt6701_read_reg(hi2c, MT6701_REG_ANGLE_14b, temp, 2);

    *angle_int = ((int16_t)temp[0] << 6) | (temp[1] >> 2);
    *angle = (float)(*angle_int) * 360 / 16384;
}

/*********************************************************************
 * MT6701 FUNCTIONS (VIA TCA9548A)
 */

// Write a single register via TCA9548A
unsigned char mt6701_write_reg_with_mux(I2C_HandleTypeDef *hi2c, uint8_t channel, uint8_t reg, uint8_t value)
{
    tca9548a_set_channel(hi2c, channel); // Select the correct channel
    return HAL_I2C_Mem_Write(hi2c, MT6701_SLAVE_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, MT6701_TIMEOUT);
}

// Read registers via TCA9548A
unsigned char mt6701_read_reg_with_mux(I2C_HandleTypeDef *hi2c, uint8_t channel, uint8_t reg, uint8_t* buf, uint16_t len)
{
    tca9548a_set_channel(hi2c, channel); // Select the correct channel
    return HAL_I2C_Mem_Read(hi2c, MT6701_SLAVE_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, MT6701_TIMEOUT);
}

// Get 14-bit angle via TCA9548A
void i2c_mt6701_get_angle_with_mux(I2C_HandleTypeDef *hi2c, uint8_t channel, int16_t *angle_int, float *angle)
{
    uint8_t temp[2];
    mt6701_read_reg_with_mux(hi2c, channel, MT6701_REG_ANGLE_14b, temp, 2);

    *angle_int = ((int16_t)temp[0] << 6) | (temp[1] >> 2);
    *angle = (float)(*angle_int) * 360 / 16384;
}
