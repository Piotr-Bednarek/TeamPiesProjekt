/**
 ******************************************************************************
 * @file    crc8.h
 * @author  Piotr Bednarek Jan Andrzejewski Mateusz Banaszak
 * @date    Jan 13, 2026
 * @brief   Nagłówek dla funkcji obliczania CRC-8.
 ******************************************************************************
 */

#ifndef INC_CRC8_H_
#define INC_CRC8_H_

#include <stdint.h>

/**
 * @brief Oblicza CRC-8 dla ciągu danych.
 * @param data Wskaźnik do danych.
 * @param len Długość danych w bajtach.
 * @return Wartość CRC-8 (0x00 - 0xFF).
 */
uint8_t CalculateCRC8(const char *data, int len);

#endif /* INC_CRC8_H_ */
