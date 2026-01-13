/**
 ******************************************************************************
 * @file    pid.c
 * @author  Piotr Bednarek Jan Andrzejewski Mateusz Banaszak
 * @date    Jan 8, 2026
 * @brief   Wrapper na bibliotekę CMSIS DSP PID.
 *
 * Plik zawiera funkcje inicjalizujące i obsługujące regulator PID
 * z wykorzystaniem zoptymalizowanej biblioteki matematycznej ARM (CMSIS DSP).
 ******************************************************************************
 */

#include "pid.h"

/**
 * @brief Inicjalizuje regulator PID.
 *        Ustawia współczynniki wzmocnienia oraz limity wyjścia (nasycenie).
 *        Wywołuje arm_pid_init_f32 aby zresetować stan wewnętrzny algorytmu.
 * @param pid Wskaźnik do struktury PID_Controller_t.
 * @param Kp Wzmocnienie proporcjonalne.
 * @param Ki Wzmocnienie całkujące.
 * @param Kd Wzmocnienie różniczkujące.
 * @param min_out Dolny limit wyjścia.
 * @param max_out Górny limit wyjścia.
 */
void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd, float min_out, float max_out) {
    pid->mode = PID_MODE_STANDARD; // Domyślnie standardowy
    pid->prev_meas = 125.0f; // Inicjalizacja na środek belki (domyślny setpoint)
    pid->Kd_user = Kd; // Zapamiętaj oryginalne Kd

    pid->instance.Kp = Kp;
    pid->instance.Ki = Ki;
    pid->instance.Kd = Kd;
    
    arm_pid_init_f32(&pid->instance, 1);
    
    // Inicjalizacja filtra błędu z wartością 0 (brak początkowego błędu)
    // to zapobiega skokowi wyjścia przy pierwszym obliczeniu
    EMA_Init(&pid->error_filter, 0.8f);
    pid->error_filter.filtered_value = 0.0f; // Rozpocznij od zerowego błędu
    pid->error_filter.initialized = 1;
    
    pid->output_min = min_out;
    pid->output_max = max_out;
}

/**
 * @brief Ustawia tryb działania pochodnej PID.
 * @param pid Wskaźnik do struktury PID_Controller_t.
 * @param mode Tryb (PID_MODE_STANDARD lub PID_MODE_DERIV_ON_MEASUREMENT).
 */
void PID_SetMode(PID_Controller_t *pid, uint8_t mode) {
    if (pid->mode != mode) {
        pid->mode = mode;
        // Wymuś aktualizację wzmocnień, aby dostosować pid->instance.Kd
        PID_UpdateGains(pid, pid->instance.Kp, pid->instance.Ki, pid->Kd_user);
    }
}

/**
 * @brief Oblicza wyjście regulatora PID dla zadanego uchybu.
 *        Funkcja oblicza błąd (setpoint - measured), a następnie wywołuje funkcję arm_pid_f32.
 *        Wynik jest ograniczany (nasycany) do zakresu [output_min, output_max].
 * @param pid Wskaźnik do struktury PID_Controller_t.
 * @param setpoint Wartość zadana.
 * @param measured Wartość mierzona.
 * @return Wartość sterująca (wyjście regulatora) po saturacji.
 */
float PID_Compute(PID_Controller_t *pid, float setpoint, float measured) {
    float error = setpoint - measured;
    
    // PRE-FILTERING: Wygładzanie błędu filtrem EMA przed podaniem do CMSIS PID
    // Redukuje to szum różniczkowania (drgania serwa) i "Derivative Kick"
    float filtered_error = EMA_Update(&pid->error_filter, error);
    
    // CMSIS PID oblicza wyjście na podstawie przefiltrowanego błędu (P+I, bez D w trybie DerivOnMeas)
    float32_t out_pi = arm_pid_f32(&pid->instance, filtered_error);
    float32_t out = out_pi;

    // Jeśli tryb Derivative on Measurement, dodaj ręcznie człon różniczkujący
    // Wzór: D = -Kd * (pomiar - poprz_pomiar)
    float d_term = 0.0f;
    if (pid->mode == PID_MODE_DERIV_ON_MEASUREMENT) {
        // Uwaga: CMSIS PID w tym trybie ma wewn. Kd = 0, więc nie liczy D z błędu.
        // Dodajemy D liczone ze zmiany pomiaru.
        d_term = -pid->Kd_user * (measured - pid->prev_meas);
        out += d_term;
    }
    pid->prev_meas = measured;
    
    // Wyjście PID jest offsetem od centrum (SERVO_CENTER = 100)
    // Saturacja do zakresu [min-100, max-100] = [-50, +50]
    float center = 100.0f;
    float max_offset = pid->output_max - center;  // +50
    float min_offset = pid->output_min - center;  // -50
    
    if (out > max_offset) {
        out = max_offset;
        // ANTI-WINDUP: Zapisz tylko P+I (bez D!) do state[2]
        // D nie powinno być akumulowane w velocity-form PID
        float32_t pi_saturated = max_offset - d_term;
        if (pi_saturated > max_offset) pi_saturated = max_offset;
        pid->instance.state[2] = pi_saturated;
    } else if (out < min_offset) {
        out = min_offset;
        // ANTI-WINDUP: Zapisz tylko P+I (bez D!)
        float32_t pi_saturated = min_offset - d_term;
        if (pi_saturated < min_offset) pi_saturated = min_offset;
        pid->instance.state[2] = pi_saturated;
    }
    
    // Dodaj centrum aby uzyskać finalny kąt serwa
    return center + out;
}

/**
 * @brief Resetuje stan regulatora PID.
 *        Zeruje całkę i historię błędów.
 * @param pid Wskaźnik do struktury PID_Controller_t.
 */
void PID_Reset(PID_Controller_t *pid) {
    arm_pid_init_f32(&pid->instance, 1);
    EMA_Init(&pid->error_filter, 0.8f);
}

/**
 * @brief Aktualizuje wzmocnienia PID bez resetowania stanu.
 *        Zachowuje całkę i historię błędów.
 * @param pid Wskaźnik do struktury PID_Controller_t.
 * @param Kp Nowe wzmocnienie proporcjonalne.
 * @param Ki Nowe wzmocnienie całkujące.
 * @param Kd Nowe wzmocnienie różniczkujące.
 */
void PID_UpdateGains(PID_Controller_t *pid, float Kp, float Ki, float Kd) {
    // Zapisz stan (całka i historia błędów)
    float32_t state0 = pid->instance.state[0];
    float32_t state1 = pid->instance.state[1];
    float32_t state2 = pid->instance.state[2];
    
    // Aktualizuj wzmocnienia
    // Aktualizuj wzmocnienia
    pid->instance.Kp = Kp;
    pid->instance.Ki = Ki;
    
    pid->Kd_user = Kd; // Zawsze aktualizuj User Kd
    
    if (pid->mode == PID_MODE_DERIV_ON_MEASUREMENT) {
        pid->instance.Kd = 0.0f; // W CMSIS wyłączamy D, liczymy ręcznie
    } else {
        pid->instance.Kd = Kd;   // Standardowy tryb
    }
    
    // Przelicz współczynniki A0, A1, A2 (to samo co arm_pid_init_f32)
    // A0 = Kp + Ki + Kd
    // A1 = -(Kp + 2*Kd)
    // A2 = Kd
    pid->instance.A0 = pid->instance.Kp + pid->instance.Ki + pid->instance.Kd;
    pid->instance.A1 = -(pid->instance.Kp + 2.0f * pid->instance.Kd);
    pid->instance.A2 = pid->instance.Kd;
    
    // Przywróć stan
    pid->instance.state[0] = state0;
    pid->instance.state[1] = state1;
    pid->instance.state[2] = state2;
}

