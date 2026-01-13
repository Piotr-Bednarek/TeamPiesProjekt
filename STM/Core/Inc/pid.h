/**
 ******************************************************************************
 * @file    pid.h
 * @author  Piotr Bednarek Jan Andrzejewski Mateusz Banaszak
 * @date    Jan 8, 2026
 * @brief   Nagłówek biblioteki regulatora PID (wrapper CMSIS DSP).
 ******************************************************************************
 */

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"
#define ARM_MATH_CM7
#include "arm_math.h"
#include "filters.h"

#define PID_MODE_STANDARD               0
#define PID_MODE_DERIV_ON_MEASUREMENT   1

typedef struct {
    arm_pid_instance_f32 instance; ///< Struktura wewnętrzna CMSIS DSP
    float output_min;              ///< Minimalna wartość wyjścia
    float output_max;              ///< Maksymalna wartość wyjścia
    EMA_Filter_t error_filter;     ///< Filtr dolnoprzepustowy dla uchybu (redukcja szumów)
    
    uint8_t mode;                  ///< Tryb działania (0=Standard, 1=DerivOnMeas)
    float prev_meas;               ///< Poprzedni pomiar (dla trybu DerivOnMeas)
    float Kd_user;                 ///< Rzeczywiste Kd ustawione przez użytkownika
} PID_Controller_t;

/**
 * @brief Inicjalizacja regulatora PID z wykorzystaniem CMSIS DSP
 * @param pid Wskaźnik na strukturę kontrolera
 * @param Kp Wzmocnienie proporcjonalne
 * @param Ki Wzmocnienie całkujące
 * @param Kd Wzmocnienie różniczkujące
 * @param min_out Minimalna wartość wyjścia (nasycenie)
 * @param max_out Maksymalna wartość wyjścia (nasycenie)
 */
void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd, float min_out, float max_out);

/**
 * @brief Ustawia tryb działania pochodnej PID
 * @param pid Wskaźnik do struktury PID_Controller_t.
 * @param mode Tryb (PID_MODE_STANDARD lub PID_MODE_DERIV_ON_MEASUREMENT)
 */
void PID_SetMode(PID_Controller_t *pid, uint8_t mode);

/**
 * @brief Oblicza wyjście regulatora PID
 * @param pid Wskaźnik na strukturę kontrolera
 * @param setpoint Wartość zadana (cel)
 * @param measured Wartość mierzona (aktualna)
 * @return Wartość sterująca (wyjście regulatora)
 */
float PID_Compute(PID_Controller_t *pid, float setpoint, float measured);

/**
 * @brief Aktualizuje wzmocnienia PID bez resetowania stanu
 * @param pid Wskaźnik na strukturę kontrolera
 * @param Kp Nowe wzmocnienie proporcjonalne
 * @param Ki Nowe wzmocnienie całkujące
 * @param Kd Nowe wzmocnienie różniczkujące
 */
void PID_UpdateGains(PID_Controller_t *pid, float Kp, float Ki, float Kd);

/**
 * @brief Resetuje stan regulatora (całkę i poprzednie błędy)
 * @param pid Wskaźnik na strukturę kontrolera
 */
void PID_Reset(PID_Controller_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
