#include "i2c.h"

// Implementation of I2C write wrapper using HAL
// Note: HAL expects address shifted left by 1 (8-bit address)
uint8_t i2c_write(uint8_t address, uint8_t* data, uint16_t count)
{
    // HAL_I2C_Master_Transmit takes address shifted by 1
    // Timeout set to 100ms
    if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(address << 1), data, count, 100) != HAL_OK)
    {
        return 0; // Error
    }
    return 1; // Success (status usually checked against HAL_OK=0, but library expects status/bool)
}

// Implementation of I2C read wrapper using HAL
uint8_t i2c_read(uint8_t address, uint8_t* data, uint16_t count)
{
    // HAL_I2C_Master_Receive takes address shifted by 1
    if (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(address << 1), data, count, 100) != HAL_OK)
    {
        return 0; // Error
    }
    return 1; // Success
}
