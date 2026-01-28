/**
 ******************************************************************************
 * @file    lqr.c
 * @author  Piotr Bednarek Jan Andrzejewski Mateusz Banaszak
 * @date    Jan 28, 2026
 * @brief   Implementacja regulatora LQR (Linear Quadratic Regulator).
 *
 * Regulator LQR z estymatorem prędkości metodą różnicową.
 * Sterowanie optymalne: u = -K1*x1 - K2*x2
 * gdzie x1 = uchyb pozycji, x2 = prędkość
 ******************************************************************************
 */

#include "lqr.h"
#include "main.h"  // Dla SERVO_CENTER

/**
 * @brief Inicjalizuje regulator LQR.
 */
void LQR_Init(LQR_Controller_t *lqr, float K1, float K2, float min_out, float max_out) {
    lqr->K1 = K1;
    lqr->K2 = K2;
    lqr->prev_position = 125.0f;  // Inicjalizacja na środek belki
    lqr->velocity = 0.0f;
    lqr->output_min = min_out;
    lqr->output_max = max_out;
}

/**
 * @brief Oblicza wyjście regulatora LQR.
 *        Implementuje prawo sterowania: u = -K1*x1 - K2*x2
 *        gdzie x1 = uchyb pozycji, x2 = prędkość (estymowana)
 */
float LQR_Compute(LQR_Controller_t *lqr, float setpoint, float measured, float dt) {
    // 1. Oblicz uchyb pozycji (x1)
    float position_error = setpoint - measured;
    
    // 2. Estymuj prędkość (x2) metodą różnic skończonych
    // velocity = dx/dt ≈ (x_current - x_prev) / dt
    float velocity = (measured - lqr->prev_position) / dt;
    lqr->velocity = velocity;  // Zapisz do struktury dla debugowania
    lqr->prev_position = measured;
    
    // 3. Prawo sterowania LQR: u = -K1*x1 - K2*x2
    // Uwaga: znak minus jest w formule LQR (minimalizacja kosztu)
    float control_signal = -lqr->K1 * position_error - lqr->K2 * velocity;
    
    // 4. Offset od centrum i saturacja
    float center = SERVO_CENTER;
    float max_offset = lqr->output_max - center;
    float min_offset = lqr->output_min - center;
    
    if (control_signal > max_offset) {
        control_signal = max_offset;
    } else if (control_signal < min_offset) {
        control_signal = min_offset;
    }
    
    // 5. Dodaj centrum aby uzyskać finalny kąt serwa
    return SERVO_CENTER + control_signal;
}

/**
 * @brief Resetuje stan regulatora LQR.
 */
void LQR_Reset(LQR_Controller_t *lqr) {
    lqr->prev_position = 125.0f;
    lqr->velocity = 0.0f;
}

/**
 * @brief Aktualizuje wzmocnienia LQR bez resetowania stanu.
 */
void LQR_UpdateGains(LQR_Controller_t *lqr, float K1, float K2) {
    lqr->K1 = K1;
    lqr->K2 = K2;
}
