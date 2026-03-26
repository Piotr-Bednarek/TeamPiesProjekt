#ifndef I2C_H_
#define I2C_H_

#include "stm32f7xx_hal.h"

// External I2C handle from main.c
extern I2C_HandleTypeDef hi2c1;

// Wrapper functions required by VL53L0X library
uint8_t i2c_write(uint8_t address, uint8_t* data, uint16_t count);
uint8_t i2c_read(uint8_t address, uint8_t* data, uint16_t count);

#endif /* I2C_H_ */
