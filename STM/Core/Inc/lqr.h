/**
 ******************************************************************************
 * @file    lqr.h
 * @author  Piotr Bednarek Jan Andrzejewski Mateusz Banaszak
 * @date    Jan 28, 2026
 * @brief   Nagłówek dla regulatora LQR (Linear Quadratic Regulator).
 *
 * Implementacja regulatora LQR z estymatorem prędkości.
 * Sterowanie: u = -K[0]*error - K[1]*velocity
 ******************************************************************************
 */

#ifndef INC_LQR_H_
#define INC_LQR_H_

#include <stdint.h>

/**
 * @brief Struktura regulatora LQR.
 */
typedef struct {
    float K1;  // Wzmocnienie dla pozycji (uchybu)
    float K2;  // Wzmocnienie dla prędkości
    
    float prev_position;  // Poprzednia pozycja do estymacji prędkości
    float velocity;       // Estymowana prędkość
    
    float output_min;  // Minimalne wyjście (offset od SERVO_CENTER)
    float output_max;  // Maksymalne wyjście (offset od SERVO_CENTER)
} LQR_Controller_t;

/**
 * @brief Inicjalizuje regulator LQR.
 * @param lqr Wskaźnik do struktury LQR_Controller_t.
 * @param K1 Wzmocnienie dla pozycji (uchybu).
 * @param K2 Wzmocnienie dla prędkości.
 * @param min_out Dolny limit wyjścia.
 * @param max_out Górny limit wyjścia.
 */
void LQR_Init(LQR_Controller_t *lqr, float K1, float K2, float min_out, float max_out);

/**
 * @brief Oblicza wyjście regulatora LQR.
 * @param lqr Wskaźnik do struktury LQR_Controller_t.
 * @param setpoint Wartość zadana.
 * @param measured Wartość mierzona.
 * @param dt Czas próbkowania w sekundach.
 * @return Wartość sterująca (kąt serwa).
 */
float LQR_Compute(LQR_Controller_t *lqr, float setpoint, float measured, float dt);

/**
 * @brief Resetuje stan regulatora LQR.
 * @param lqr Wskaźnik do struktury LQR_Controller_t.
 */
void LQR_Reset(LQR_Controller_t *lqr);

/**
 * @brief Aktualizuje wzmocnienia LQR bez resetowania stanu.
 * @param lqr Wskaźnik do struktury LQR_Controller_t.
 * @param K1 Nowe wzmocnienie dla pozycji.
 * @param K2 Nowe wzmocnienie dla prędkości.
 */
void LQR_UpdateGains(LQR_Controller_t *lqr, float K1, float K2);

#endif /* INC_LQR_H_ */
