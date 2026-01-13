/**
 ******************************************************************************
 * @file    leds.c
 * @author  Piotr Bednarek Jan Andrzejewski Mateusz Banaszak
 * @date    Jan 13, 2026
 * @brief   Implementacja modułu wyświetlania błędu na LEDach.
 *
 * Wizualizacja uchybu regulacji na 5 diodach LED (PE2-PE6):
 * - PE2, PE6: Czerwone (duży błąd)
 * - PE3, PE5: Żółte (średni błąd)
 * - PE4: Zielona (mały błąd - OK)
 ******************************************************************************
 */

#include "leds.h"
#include "main.h"

/**
 * @brief Aktualizuje 5 LEDów na podstawie wartości uchybu.
 */
void UpdateErrorLEDs_5LED(float error) {
	// Wyłącz wszystkie LEDy (PE2..PE6)
	HAL_GPIO_WritePin(GPIOE, LED_ERR_1_Pin | LED_ERR_2_Pin | LED_ERR_3_Pin | 
	                         LED_ERR_4_Pin | LED_ERR_5_Pin, GPIO_PIN_RESET);
	
	// Progi: 
	// 1% z 250mm = 2.5mm
	// 5% z 250mm = 12.5mm
	float limit_green = 2.5f;
	float limit_yellow = 12.5f;
	
	if (error < -limit_yellow) {
		// Duży ujemny -> Czerwona (PE2 - LED_ERR_1)
		HAL_GPIO_WritePin(GPIOE, LED_ERR_1_Pin, GPIO_PIN_SET);
	} else if (error < -limit_green) {
		// Średni ujemny -> Żółta (PE3 - LED_ERR_2)
		HAL_GPIO_WritePin(GPIOE, LED_ERR_2_Pin, GPIO_PIN_SET);
	} else if (error <= limit_green) { 
		// Mały błąd (+/- 2.5mm) -> Zielona (PE4 - LED_ERR_3)
		HAL_GPIO_WritePin(GPIOE, LED_ERR_3_Pin, GPIO_PIN_SET);
	} else if (error <= limit_yellow) {
		// Średni dodatni -> Żółta (PE5 - LED_ERR_4)
		HAL_GPIO_WritePin(GPIOE, LED_ERR_4_Pin, GPIO_PIN_SET);
	} else {
		// Duży dodatni -> Czerwona (PE6 - LED_ERR_5)
		HAL_GPIO_WritePin(GPIOE, LED_ERR_5_Pin, GPIO_PIN_SET);
	}
}

/**
 * @brief Funkcja testowa - miga LED co 500ms.
 */
void TestLED(void) {
	static uint32_t last_toggle = 0;
	
	if (HAL_GetTick() - last_toggle > 500) {
		HAL_GPIO_TogglePin(LED_ERR_PORT, LED_ERR_1_Pin);
		last_toggle = HAL_GetTick();
	}
}
