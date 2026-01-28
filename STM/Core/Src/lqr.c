/**
 ******************************************************************************
 * @file    lqr.c
 * @author  Piotr Bednarek Jan Andrzejewski Mateusz Banaszak
 * @date    Jan 28, 2026
 * @brief   Implementacja regulatora LQR (Linear Quadratic Regulator).
 *
 * Model 3-stanowy dla serwa pozycyjnego:
 * Stan x = [pozycja_kulki, prędkość_kulki, kąt_belki]
 * Sterowanie optymalne: u = -K1*x - K2*v - K3*theta
 ******************************************************************************
 */

#include "lqr.h"
#include "main.h"  // Dla SERVO_CENTER

/**
 * @brief Inicjalizuje regulator LQR.
 */
void LQR_Init(LQR_Controller_t *lqr, float K1, float K2, float K3, float min_out, float max_out) {
    lqr->K1 = K1;
    lqr->K2 = K2;
    lqr->K3 = K3;
    lqr->prev_position = 125.0f;
    lqr->velocity = 0.0f;
    lqr->prev_output = SERVO_CENTER;
    lqr->output_min = min_out;
    lqr->output_max = max_out;
}

/**
 * @brief Oblicza wyjście regulatora LQR (3-stanowy).
 *        Implementuje prawo sterowania: u = -K1*x - K2*v - K3*theta
 *        gdzie x = uchyb pozycji, v = prędkość, theta = kąt belki
 */
float LQR_Compute(LQR_Controller_t *lqr, float setpoint, float measured, float beam_angle, float dt) {
    float position_error_mm = setpoint - measured;
    if (position_error_mm > -LQR_ERROR_DEADBAND && position_error_mm < LQR_ERROR_DEADBAND) {
        position_error_mm = 0.0f;
    }
    float position_error_m = position_error_mm / 1000.0f;
    
    float velocity_mm = (measured - lqr->prev_position) / dt;
    if (velocity_mm > -LQR_VELOCITY_DEADBAND && velocity_mm < LQR_VELOCITY_DEADBAND) {
        velocity_mm = 0.0f;
    }
    float velocity_m = velocity_mm / 1000.0f;
    lqr->velocity = velocity_mm;
    lqr->prev_position = measured;
    
    float theta = beam_angle;
    
    // u = K1*error - K2*velocity - K3*theta
    float control_rad = lqr->K1 * position_error_m 
                        - lqr->K2 * velocity_m 
                        - lqr->K3 * theta;
    
    // Konwersja rad -> servo units (1° ≈ 2.33 units)
    float control_deg = control_rad * (180.0f / 3.14159f);
    float control_servo = control_deg * 2.33f;
    
    // Saturacja
    float max_offset = lqr->output_max - SERVO_CENTER;
    float min_offset = lqr->output_min - SERVO_CENTER;
    
    if (control_servo > max_offset) {
        control_servo = max_offset;
    } else if (control_servo < min_offset) {
        control_servo = min_offset;
    }
    
    float raw_output = SERVO_CENTER + control_servo;
    
    // Smoothing EMA
    float smoothed_output = LQR_SMOOTHING_ALPHA * raw_output 
                          + (1.0f - LQR_SMOOTHING_ALPHA) * lqr->prev_output;
    lqr->prev_output = smoothed_output;
    
    return smoothed_output;
}

/**
 * @brief Resetuje stan regulatora LQR.
 */
void LQR_Reset(LQR_Controller_t *lqr) {
    lqr->prev_position = 125.0f;
    lqr->velocity = 0.0f;
    lqr->prev_output = SERVO_CENTER;
}

/**
 * @brief Aktualizuje wzmocnienia LQR bez resetowania stanu.
 */
void LQR_UpdateGains(LQR_Controller_t *lqr, float K1, float K2, float K3) {
    lqr->K1 = K1;
    lqr->K2 = K2;
    lqr->K3 = K3;
}
