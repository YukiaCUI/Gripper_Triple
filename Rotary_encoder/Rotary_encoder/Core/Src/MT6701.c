#include "MT6701.h"

unsigned char mt6701_write_reg(unsigned char reg, unsigned char value)
{
	return HAL_I2C_Mem_Write(&hi2c1, MT6701_SLAVE_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, MT6701_Timeout);
}

unsigned char mt6701_write_regs(unsigned char reg, unsigned char *value, unsigned char len)
{
	return HAL_I2C_Mem_Write(&hi2c1, MT6701_SLAVE_ADDR, reg, I2C_MEMADD_SIZE_8BIT, value, len, MT6701_Timeout);
}

unsigned char mt6701_read_reg(I2C_HandleTypeDef *hi2c, unsigned char reg, unsigned char* buf, unsigned short len)
{
    return HAL_I2C_Mem_Read(hi2c, MT6701_SLAVE_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, MT6701_Timeout);
}

void mt6701_delay(unsigned int ms)
{
	HAL_Delay(ms);
}

// 14Bit角度信息，存储在0x03[13:6]、0x04[5:0]两个寄存器中，高位在前，原始读数0~16383，对应0-360°
void i2c_mt6701_get_angle(I2C_HandleTypeDef *hi2c, int16_t *angle_int, float *angle)
{
    uint8_t temp[2];
    mt6701_read_reg(hi2c, MT6701_REG_ANGLE_14b, temp, 2);

    *angle_int = ((int16_t)temp[0] << 6) | (temp[1] >> 2);
    *angle = (float)*angle_int * 360 / 16384;
}
