/**
 ******************************************************************************
 * @file    lqr.h
 * @author  Piotr Bednarek Jan Andrzejewski Mateusz Banaszak
 * @date    Jan 28, 2026
 * @brief   Nagłówek dla regulatora LQR (Linear Quadratic Regulator).
 *
 * Model 3-stanowy dla serwa pozycyjnego:
 * Stan x = [pozycja_kulki, prędkość_kulki, kąt_belki]
 * Sterowanie: u = -K1*x - K2*v - K3*theta
 ******************************************************************************
 */

#ifndef INC_LQR_H_
#define INC_LQR_H_

#include <stdint.h>

/**
 * @brief Struktura regulatora LQR (3-stanowy).
 */
typedef struct {
    float K1;  // Wzmocnienie dla pozycji (uchybu)
    float K2;  // Wzmocnienie dla prędkości
    float K3;  // Wzmocnienie dla kąta belki
    
    float prev_position;  // Poprzednia pozycja do estymacji prędkości
    float velocity;       // Estymowana prędkość
    float prev_output;    // Poprzednie wyjście (do smoothing)
    
    float output_min;  // Minimalne wyjście (offset od SERVO_CENTER)
    float output_max;  // Maksymalne wyjście (offset od SERVO_CENTER)
} LQR_Controller_t;

// Parametry smoothing i deadband
#define LQR_SMOOTHING_ALPHA  0.3f   // EMA alpha (0.1=wolny, 0.5=szybki)
#define LQR_ERROR_DEADBAND   3.0f   // Strefa nieczułości dla błędu [mm]
#define LQR_VELOCITY_DEADBAND 5.0f  // Strefa nieczułości dla prędkości [mm/s]

/**
 * @brief Inicjalizuje regulator LQR.
 * @param lqr Wskaźnik do struktury LQR_Controller_t.
 * @param K1 Wzmocnienie dla pozycji (uchybu).
 * @param K2 Wzmocnienie dla prędkości.
 * @param K3 Wzmocnienie dla kąta belki.
 * @param min_out Dolny limit wyjścia.
 * @param max_out Górny limit wyjścia.
 */
void LQR_Init(LQR_Controller_t *lqr, float K1, float K2, float K3, float min_out, float max_out);

/**
 * @brief Oblicza wyjście regulatora LQR (3-stanowy).
 * @param lqr Wskaźnik do struktury LQR_Controller_t.
 * @param setpoint Wartość zadana pozycji.
 * @param measured Zmierzona pozycja kulki.
 * @param beam_angle Kąt belki w radianach (z systemu wizyjnego).
 * @param dt Czas próbkowania w sekundach.
 * @return Wartość sterująca (kąt serwa).
 */
float LQR_Compute(LQR_Controller_t *lqr, float setpoint, float measured, float beam_angle, float dt);

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
 * @param K3 Nowe wzmocnienie dla kąta belki.
 */
void LQR_UpdateGains(LQR_Controller_t *lqr, float K1, float K2, float K3);

#endif /* INC_LQR_H_ */
