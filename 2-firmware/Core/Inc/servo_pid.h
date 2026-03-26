/**
 ******************************************************************************
 * @file    servo_pid.h
 * @author  Piotr Bednarek Jan Andrzejewski Mateusz Banaszak
 * @date    Jan 13, 2026
 * @brief   Nagłówek dla regulatora PID serwa z Anti-Windup.
 *
 * Zawiera strukturę ServoPID_Controller oraz deklaracje funkcji
 * inicjalizacji i obliczania wyjścia PID.
 ******************************************************************************
 */

#ifndef INC_SERVO_PID_H_
#define INC_SERVO_PID_H_

#include "filters.h"

// --- Parametry PID ---
typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float prevError;
	float integral;
	float prevMeasurement;  // Do obliczania pochodnej na podstawie pomiaru (Derivative-on-Measurement)
	EMA_Filter_t d_filter;  // Filtr dolnoprzepustowy dla członu różniczkującego
} ServoPID_Controller;

/**
 * @brief Inicjalizuje regulator ServoPID.
 * @param pid Wskaźnik do struktury ServoPID_Controller.
 */
void ServoPID_Init(ServoPID_Controller *pid);

/**
 * @brief Oblicza wyjście regulatora PID z Anti-Windup.
 * @param pid Wskaźnik do struktury ServoPID_Controller.
 * @param error Uchyb regulacji (setpoint - measured).
 * @param measurement Aktualna wartość mierzona (do Derivative-on-Measurement).
 * @return Kąt serwa (wyjście regulatora).
 */
float ServoPID_Compute(ServoPID_Controller *pid, float error, float measurement);

/**
 * @brief Resetuje stan regulatora (integral i poprzednie wartości).
 * @param pid Wskaźnik do struktury ServoPID_Controller.
 */
void ServoPID_Reset(ServoPID_Controller *pid);

#endif /* INC_SERVO_PID_H_ */
