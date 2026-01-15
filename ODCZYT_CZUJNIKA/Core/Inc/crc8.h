/**
 ******************************************************************************
 * @file    crc8.h
 * @brief   Nagłówek dla funkcji obliczania CRC-8.
 ******************************************************************************
 */

#ifndef INC_CRC8_H_
#define INC_CRC8_H_

#include <stdint.h>

uint8_t CalculateCRC8(const char *data, int len);

#endif /* INC_CRC8_H_ */
