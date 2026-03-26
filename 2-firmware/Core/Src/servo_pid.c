/**
 ******************************************************************************
 * @file    servo_pid.c
 * @author  Piotr Bednarek Jan Andrzejewski Mateusz Banaszak
 * @date    Jan 13, 2026
 * @brief   Implementacja regulatora PID serwa z Anti-Windup.
 *
 * Zawiera funkcje inicjalizacji i obliczania wyjścia PID z:
 * - Anti-Windup (Clamping)
 * - Derivative-on-Measurement (unikanie derivative kick)
 * - Filtr dolnoprzepustowy na członie różniczkującym
 ******************************************************************************
 */

#include "servo_pid.h"
#include "main.h"  // Dla SERVO_CENTER i limitów

// Zewnętrzne zmienne globalne (z main.c)
extern volatile float g_Kp;
extern volatile float g_Ki;
extern volatile float g_Kd;

// Stałe konfiguracyjne (zdefiniowane w main.h)

/**
 * @brief Inicjalizuje regulator ServoPID.
 */
void ServoPID_Init(ServoPID_Controller *pid) {
	pid->Kp = 0.0f;
	pid->Ki = 0.0f;
	pid->Kd = 0.0f;
	pid->prevError = 0.0f;
	pid->integral = 0.0f;
	pid->prevMeasurement = 0.0f;
	EMA_Init(&pid->d_filter, D_FILTER_ALPHA);
}

/**
 * @brief Oblicza wyjście regulatora PID z Anti-Windup.
 *        Implementacja używa:
 *        - Derivative-on-Measurement (zapobiega derivative kick przy zmianach setpointu)
 *        - Clamping Anti-Windup (zapobiega wind-up całki przy nasyceniu)
 */
float ServoPID_Compute(ServoPID_Controller *pid, float error, float measurement) {
	// Pobierz aktualne wzmocnienia z globalnych zmiennych
	pid->Kp = g_Kp;
	pid->Ki = g_Ki;
	pid->Kd = g_Kd;

	// D (Derivative on Measurement)
	float D = 0.0f;
	float raw_derivative = -(measurement - pid->prevMeasurement);
	if (raw_derivative > -D_DEADBAND && raw_derivative < D_DEADBAND) {
		raw_derivative = 0.0f;
	}
	float filtered_derivative = EMA_Update(&pid->d_filter, raw_derivative);
	D = pid->Kd * filtered_derivative;
	pid->prevMeasurement = measurement;

	// P (Proportional)
	float P = pid->Kp * error;

	// Anti-Windup (Clamping)
	float old_I = pid->Ki * pid->integral;
	float tentative_output = SERVO_CENTER + P + old_I + D;

	int saturated = 0;
	if (tentative_output > SERVO_MAX_LIMIT) {
		saturated = 1;
	} else if (tentative_output < SERVO_MIN_LIMIT) {
		saturated = -1;
	}

	if (saturated == 0) {
		pid->integral += error;
	} else if (saturated == 1 && error < 0) {
		pid->integral += error;
	} else if (saturated == -1 && error > 0) {
		pid->integral += error;
	} else {
		pid->integral *= 0.9f;
	}

	if (pid->integral > 100.0f)
		pid->integral = 100.0f;
	if (pid->integral < -100.0f)
		pid->integral = -100.0f;

	// Output
	float new_I = pid->Ki * pid->integral;
	float output = SERVO_CENTER + P + new_I + D;

	pid->prevError = error;

	return output;
}

/**
 * @brief Resetuje stan regulatora.
 */
void ServoPID_Reset(ServoPID_Controller *pid) {
	pid->prevError = 0.0f;
	pid->integral = 0.0f;
	pid->prevMeasurement = 0.0f;
	EMA_Init(&pid->d_filter, D_FILTER_ALPHA);
}
