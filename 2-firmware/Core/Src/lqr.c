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
 * 
 * WYJŚCIE: Kąt serwa w STOPNIACH (środek = 0°)
 ******************************************************************************
 */

#include "lqr.h"
#include "main.h"

/**
 * @brief Inicjalizuje regulator LQR.
 * @param min_out: Minimalny kąt wyjściowy w stopniach (np. -30°)
 * @param max_out: Maksymalny kąt wyjściowy w stopniach (np. +30°)
 */
void LQR_Init(LQR_Controller_t *lqr, float K1, float K2, float K3, float min_out, float max_out) {
    lqr->K1 = K1;
    lqr->K2 = K2;
    lqr->K3 = K3;
    lqr->prev_position = 125.0f;
    lqr->velocity = 0.0f;
    lqr->prev_output = 0.0f;  // Środek = 0°
    lqr->output_min = min_out;
    lqr->output_max = max_out;
}

/**
 * @brief Oblicza wyjście regulatora LQR (3-stanowy).
 *        Implementuje prawo sterowania: u = K1*error - K2*velocity - K3*theta
 * @return Kąt serwa w STOPNIACH (środek = 0°)
 */
float LQR_Compute(LQR_Controller_t *lqr, float setpoint, float measured, float beam_angle, float dt) {
    // Błąd pozycji [mm] -> [m]
    float position_error_mm = setpoint - measured;
    if (position_error_mm > -LQR_ERROR_DEADBAND && position_error_mm < LQR_ERROR_DEADBAND) {
        position_error_mm = 0.0f;
    }
    float position_error_m = position_error_mm / 1000.0f;
    
    // Prędkość kulki [mm/s] -> [m/s]
    float velocity_mm = (measured - lqr->prev_position) / dt;
    if (velocity_mm > -LQR_VELOCITY_DEADBAND && velocity_mm < LQR_VELOCITY_DEADBAND) {
        velocity_mm = 0.0f;
    }
    float velocity_m = velocity_mm / 1000.0f;
    lqr->velocity = velocity_mm;
    lqr->prev_position = measured;
    
    // Kąt belki [rad] (przekazany z zewnątrz)
    float theta = beam_angle;
    
    // Prawo sterowania: u = K1*error - K2*velocity - K3*theta
    // Wynik w RADIANACH
    float control_rad = lqr->K1 * position_error_m 
                        - lqr->K2 * velocity_m 
                        - lqr->K3 * theta;
    
    // Konwersja rad -> stopnie
    float control_deg = control_rad * (180.0f / 3.14159f);
    
    // Saturacja do limitów roboczych [stopnie]
    if (control_deg > lqr->output_max) {
        control_deg = lqr->output_max;
    } else if (control_deg < lqr->output_min) {
        control_deg = lqr->output_min;
    }
    
    // === KOMPENSACJA TARCIA STATYCZNEGO (Stiction) ===
    float friction_comp = 0.0f;
    const float MIN_ANGLE_TO_MOVE_DEG = 1.0f;  // Minimalny kąt do pokonania tarcia [stopnie]
    const float CONTROL_THRESHOLD_DEG = 0.5f; // Próg wykrywania intencji ruchu [stopnie]
    
    if (control_deg > CONTROL_THRESHOLD_DEG) {
        friction_comp = MIN_ANGLE_TO_MOVE_DEG;
    } else if (control_deg < -CONTROL_THRESHOLD_DEG) {
        friction_comp = -MIN_ANGLE_TO_MOVE_DEG;
    }
    
    float compensated_deg = control_deg + friction_comp;
    
    // Re-saturacja po kompensacji
    if (compensated_deg > lqr->output_max) {
        compensated_deg = lqr->output_max;
    } else if (compensated_deg < lqr->output_min) {
        compensated_deg = lqr->output_min;
    }
    
    // Smoothing EMA
    float smoothed_output = LQR_SMOOTHING_ALPHA * compensated_deg 
                          + (1.0f - LQR_SMOOTHING_ALPHA) * lqr->prev_output;
    lqr->prev_output = smoothed_output;
    
    return smoothed_output;  // Wyjście w STOPNIACH
}

/**
 * @brief Resetuje stan regulatora LQR.
 */
void LQR_Reset(LQR_Controller_t *lqr) {
    lqr->prev_position = 125.0f;
    lqr->velocity = 0.0f;
    lqr->prev_output = 0.0f;  // Środek = 0°
}

/**
 * @brief Aktualizuje wzmocnienia LQR bez resetowania stanu.
 */
void LQR_UpdateGains(LQR_Controller_t *lqr, float K1, float K2, float K3) {
    lqr->K1 = K1;
    lqr->K2 = K2;
    lqr->K3 = K3;
}
