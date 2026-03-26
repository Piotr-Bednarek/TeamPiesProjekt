/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_ERR_1_Pin GPIO_PIN_2
#define LED_ERR_1_GPIO_Port GPIOE
#define LED_ERR_2_Pin GPIO_PIN_3
#define LED_ERR_2_GPIO_Port GPIOE
#define LED_ERR_3_Pin GPIO_PIN_4
#define LED_ERR_3_GPIO_Port GPIOE
#define LED_ERR_4_Pin GPIO_PIN_5
#define LED_ERR_4_GPIO_Port GPIOE
#define LED_ERR_5_Pin GPIO_PIN_6
#define LED_ERR_5_GPIO_Port GPIOE
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define CALIB_START_BTN_Pin GPIO_PIN_1
#define CALIB_START_BTN_GPIO_Port GPIOB
#define CALIB_MID_BTN_Pin GPIO_PIN_2
#define CALIB_MID_BTN_GPIO_Port GPIOB
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define CALIB_END_BTN_Pin GPIO_PIN_4
#define CALIB_END_BTN_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
// --- Global System Settings ---
#define PID_DT_MS          30
#define SETPOINT_DEFAULT   125.0f
#define BALL_RADIUS        20
#define AVG_ERR_SAMPLES    35

// --- Logic Modes ---
#define SENSOR_TEST_MODE   0
#define USE_CALIBRATION    1

// --- Servo Configuration (in DEGREES, center = 0°) ---
#define SERVO_CENTER_DEG   0.0f      // Środek = poziomo
#define SERVO_MIN_DEG     -30.0f     // Limit lewy (stare 70)
#define SERVO_MAX_DEG      30.0f     // Limit prawy (stare 130)
#define SERVO_SLEW_RATE    200.0f    // Stopni na sekundę (było 1000 jednostek/s)
#define SERVO_ANGLE_DEADBAND 0.1f    // Stopnie
#define SERVO_SMOOTHING_SIZE 1

// --- Servo Hardware (PWM) ---
#define SERVO_PWM_MIN      500       // 500μs = -90°
#define SERVO_PWM_CENTER   1500      // 1500μs = 0°
#define SERVO_PWM_MAX      2500      // 2500μs = +90°
#define SERVO_PWM_RANGE    90.0f     // ±90° zakres fizyczny serwa

// --- Compatibility macros (for gradual migration) ---
// Stare nazwy -> nowe wartości (do usunięcia po pełnej migracji)
#define SERVO_CENTER       SERVO_CENTER_DEG   // 0° (było 100)
#define SERVO_MIN_LIMIT    SERVO_MIN_DEG      // -30° (było 70)  
#define SERVO_MAX_LIMIT    SERVO_MAX_DEG      // +30° (było 130)

// --- PID & Filtration ---
#define D_DEADBAND         0.1f
#define D_FILTER_ALPHA     0.5f

// --- LED Error Bar ---
#define LED_ERR_PORT      GPIOE
#define LED_ERR_1_Pin     GPIO_PIN_2
#define LED_ERR_2_Pin     GPIO_PIN_3
#define LED_ERR_3_Pin     GPIO_PIN_4
#define LED_ERR_4_Pin     GPIO_PIN_5
#define LED_ERR_5_Pin     GPIO_PIN_6
#define LED_ERR_6_Pin     GPIO_PIN_7
#define LED_ERR_7_Pin     GPIO_PIN_8
#define LED_ERR_8_Pin     GPIO_PIN_9
#define LED_ERR_9_Pin     GPIO_PIN_10
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
