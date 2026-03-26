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
    float K1;
    float K2;
    float K3;
    float prev_position;
    float velocity;
    float prev_output;
    float output_min;
    float output_max;
} LQR_Controller_t;

#define LQR_SMOOTHING_ALPHA   0.3f
#define LQR_ERROR_DEADBAND    3.0f
#define LQR_VELOCITY_DEADBAND 5.0f

void LQR_Init(LQR_Controller_t *lqr, float K1, float K2, float K3, float min_out, float max_out);
float LQR_Compute(LQR_Controller_t *lqr, float setpoint, float measured, float beam_angle, float dt);
void LQR_Reset(LQR_Controller_t *lqr);
void LQR_UpdateGains(LQR_Controller_t *lqr, float K1, float K2, float K3);

#endif /* INC_LQR_H_ */
