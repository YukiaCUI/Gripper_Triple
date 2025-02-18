#ifndef __AS5600_H__
#define __AS5600_H__

#include "stm32f1xx_hal.h"
#include "i2c.h"
#include <stdio.h>


#define AS5600_SLAVE_ADDR         0x36<<1
#define AS5600_Timeout            50

#define AS5600_REG_ANGLE_12b      0x0C    // 12Bit角度信息，存储在0x03[11:8]、0x04[7:0]两个寄存器中，高位在前，原始读数0~16383

#define as5600_log		printf


void i2c_as5600_get_angle(int16_t *angle_int, float *angle);

unsigned char as5600_write_reg_with_mux(I2C_HandleTypeDef *hi2c, uint8_t channel, unsigned char reg, unsigned char value);
unsigned char as5600_write_regs_with_mux(I2C_HandleTypeDef *hi2c, uint8_t channel, unsigned char reg, unsigned char *value, unsigned char len);
unsigned char as5600_read_reg_with_mux(I2C_HandleTypeDef *hi2c, uint8_t channel, unsigned char reg, unsigned char* buf, unsigned short len);
void i2c_as5600_get_angle_with_mux(I2C_HandleTypeDef *hi2c, uint8_t channel, int16_t *angle_int, float *angle);


#endif

