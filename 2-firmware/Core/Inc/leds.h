/**
 ******************************************************************************
 * @file    leds.h
 * @author  Piotr Bednarek Jan Andrzejewski Mateusz Banaszak
 * @date    Jan 13, 2026
 * @brief   Nagłówek dla modułu wyświetlania błędu na LEDach.
 ******************************************************************************
 */

#ifndef INC_LEDS_H_
#define INC_LEDS_H_

/**
 * @brief Aktualizuje 5 LEDów (R-Ż-Z-Ż-R) na podstawie wartości uchybu.
 *        - Mały uchyb: zielona (środek)
 *        - Średni uchyb: żółte (boki)
 *        - Duży uchyb: czerwone (skrajne)
 * @param error Wartość uchybu w mm (może być ujemna).
 */
void UpdateErrorLEDs_5LED(float error);

/**
 * @brief Funkcja testowa - miga LED co 500ms.
 */
void TestLED(void);

#endif /* INC_LEDS_H_ */
