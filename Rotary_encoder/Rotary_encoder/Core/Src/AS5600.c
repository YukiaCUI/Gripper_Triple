#include "AS5600.h"

unsigned char as5600_write_reg(unsigned char reg, unsigned char value)
{
	return HAL_I2C_Mem_Write(&hi2c1, AS5600_SLAVE_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, AS5600_Timeout);
}

unsigned char as5600_write_regs(unsigned char reg, unsigned char *value, unsigned char len)
{
	return HAL_I2C_Mem_Write(&hi2c1, AS5600_SLAVE_ADDR, reg, I2C_MEMADD_SIZE_8BIT, value, len, AS5600_Timeout);
}

unsigned char as5600_read_reg(unsigned char reg, unsigned char* buf, unsigned short len)
{
	return HAL_I2C_Mem_Read(&hi2c1, AS5600_SLAVE_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, AS5600_Timeout);
}

void as5600_delay(unsigned int ms)
{
	HAL_Delay(ms);
}

// 12Bit角度信息，存储在0x03[11:8]、0x04[7:0]两个寄存器中，高位在前，原始读数0~4096
void i2c_as5600_get_angle(int16_t *angle_int, float *angle)
{
    uint8_t temp[2];
    as5600_read_reg(AS5600_REG_ANGLE_12b, temp, 2);

    *angle_int = ((int16_t)temp[0] << 8) | temp[1];
    *angle = (float)*angle_int * 360 / 4096;
}

