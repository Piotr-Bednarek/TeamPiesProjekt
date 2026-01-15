/**
 ******************************************************************************
 * @file    crc8.c
 * @brief   Implementacja funkcji obliczania CRC-8.
 *          Używa wielomianu 0x07 (x^8 + x^2 + x + 1).
 ******************************************************************************
 */

#include "crc8.h"

uint8_t CalculateCRC8(const char *data, int len) {
	uint8_t crc = 0x00;
	for (int i = 0; i < len; i++) {
		crc ^= data[i];
		for (uint8_t j = 0; j < 8; j++) {
			if (crc & 0x80)
				crc = (crc << 1) ^ 0x07;
			else
				crc <<= 1;
		}
	}
	return crc;
}
