/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "VL53L0X.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "filters.h"
#include "calibration.h"
#include "servo_pid.h"
#include "lqr.h"
#include "crc8.h"
#include "leds.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2007c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2007c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2007c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection"))); /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

ADC_HandleTypeDef hadc1;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osThreadId defaultTaskHandle;
osThreadId ControlTaskHandle;
/* USER CODE BEGIN PV */
uint16_t distance;
char msg[64];

// --- DMA UART RX ---
#define DMA_RX_BUFFER_SIZE 128
uint8_t dma_rx_buffer[DMA_RX_BUFFER_SIZE];
volatile uint16_t dma_rx_head = 0;  // Wskaźnik przetwarzania

// Bufor na komendy/ramki
uint8_t cmd_buffer[64];
volatile uint8_t cmd_len = 0;
volatile uint8_t cmd_received = 0;

// Dane z systemu wizyjnego (Python)
volatile float g_vision_ball_pos = 125.0f;   // Pozycja piłeczki [mm] z OpenCV
volatile float g_vision_beam_angle = 0.0f;   // Kąt belki [stopnie] z OpenCV
volatile uint8_t g_vision_data_valid = 0;    // Flaga: czy dane są aktualne
volatile uint32_t g_vision_last_update = 0;  // Timestamp ostatniej aktualizacji

// Stare zmienne obsługi UART (do usunięcia po pełnej migracji)
uint8_t rx_byte;
uint8_t rx_buffer[64];
volatile uint8_t rx_idx = 0;

// Główne zmienne sterujące
volatile float g_setpoint = 125.0f;
volatile float g_Kp = 0.44f;
volatile float g_Ki = 0.0053f;
volatile float g_Kd = 5.0f;

// Zmienne kalibracji
// volatile uint8_t calibration_mode = 0;
// volatile float cal_raw_min = 9999.0f;
// volatile float cal_raw_max = 0.0f;
// volatile uint32_t cal_start_time = 0;

// // Domyślne punkty kalibracyjne
// volatile float sensor_min = 50.0f;
// volatile float sensor_max = 220.0f;
// volatile float sensor_middle = 115.0f;

// Bufor średniego uchybu
float err_buffer[AVG_ERR_SAMPLES];
uint8_t err_idx = 0;
float err_sum = 0.0f;

// Tryb sterowania: 0 = GUI (Manual), 1 = Analog (Potencjometr)
volatile uint8_t control_mode = 0;



// Start/Stop regulatora: 0 = wyłączony, 1 = włączony (domyślnie)
volatile uint8_t g_regulator_enabled = 1;


// Kontroler ServoPID (nasza implementacja) - domyślny
ServoPID_Controller g_servo_pid;

// Kontroler LQR (model 3-stanowy)
LQR_Controller_t g_lqr_ctrl;
volatile float g_K1 = 1.00f;
volatile float g_K2 = 0.75f;
volatile float g_K3 = 0.38f;

// Tryb regulacji: 0 = Custom PID (servo_pid, domyślny), 1 = LQR
volatile uint8_t g_pid_mode = 0;

// Debugging
volatile uint32_t g_adc_raw = 0;
volatile float g_pot_setpoint = 0.0f;

// Zmienna współdzielona do wizualizacji błędu na LED
volatile float g_current_error = 0.0f;

// Manual control angle (when regulator is disabled) - in DEGREES
volatile float g_manual_servo_angle_deg = SERVO_CENTER_DEG;

#include "signal.h" // Generated by Matlab

// --- ID TEST VARIABLES ---
volatile uint8_t g_test_running = 0;
volatile uint32_t g_test_index = 0;

// Sampling period (ms) - can be changed via UART command T:xx
volatile uint32_t g_pid_dt_ms = PID_DT_MS;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void const *argument);
void StartControlTask(void const *argument);

/* USER CODE BEGIN PFP */

void SetServoAngleDeg(float degrees);
void SetServoAngleSmoothDeg(float target_deg);
extern float g_current_hw_angle_deg;
// Funkcje PID, CRC8, LEDs przeniesione do osobnych modułów

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// --- Funkcje DMA UART (bez przerwań na każdy bajt) ---
// Parsowanie ramki wizyjnej: "V:125.5;B:2.3;C:XX\n"
// V = pozycja piłeczki [mm], B = kąt belki [stopnie], C = CRC8
void ParseVisionFrame(const char *frame) {
	float ball_pos = -1.0f;
	float beam_angle = 0.0f;
	uint8_t crc_received = 0;
	int crc_found = 0;

	// Znajdź CRC na końcu
	const char *crc_ptr = strstr(frame, ";C:");
	if (crc_ptr != NULL) {
		crc_received = (uint8_t) strtol(crc_ptr + 3, NULL, 16);
		crc_found = 1;
	}

	// Parsuj pola
	const char *ptr = frame;
	while (*ptr) {
		if (ptr[0] == 'V' && ptr[1] == ':') {
			ball_pos = atof(ptr + 2);
		} else if (ptr[0] == 'B' && ptr[1] == ':') {
			beam_angle = atof(ptr + 2);
		}
		// Przejdź do następnego pola
		while (*ptr && *ptr != ';')
			ptr++;
		if (*ptr == ';')
			ptr++;
	}

	if (crc_found && ball_pos >= 0.0f) {
		int data_len = (crc_ptr != NULL) ? (int) (crc_ptr - frame) : strlen(frame);
		uint8_t crc_calc = CalculateCRC8(frame, data_len);

		if (crc_calc == crc_received) {
			// Dane poprawne - zapisz
			g_vision_ball_pos = ball_pos;
			g_vision_beam_angle = beam_angle;
			g_vision_data_valid = 1;
			g_vision_last_update = HAL_GetTick();
		}
	} else if (ball_pos >= 0.0f) {
		// Brak CRC ale dane wyglądają ok - zaakceptuj (dla testów)
		g_vision_ball_pos = ball_pos;
		g_vision_beam_angle = beam_angle;
		g_vision_data_valid = 1;
		g_vision_last_update = HAL_GetTick();
	}
}

// Przetwarzanie bufora DMA - wywoływane w głównej pętli (polling)
void ProcessDmaRxBuffer(void) {
	// Pobierz aktualną pozycję DMA (ile bajtów pozostało do końca bufora)
	uint16_t dma_tail = DMA_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);

	while (dma_rx_head != dma_tail) {
		uint8_t byte = dma_rx_buffer[dma_rx_head];
		dma_rx_head = (dma_rx_head + 1) % DMA_RX_BUFFER_SIZE;

		if (byte == '\n' || byte == '\r') {
			if (cmd_len > 0) {
				cmd_buffer[cmd_len] = '\0';

				// Sprawdź typ ramki
				if (cmd_buffer[0] == 'V' && cmd_buffer[1] == ':') {
					// Ramka z systemu wizyjnego
					ParseVisionFrame((char*) cmd_buffer);
				} else {
					// Standardowa komenda (S:, P:, I:, D:, itd.)
					memcpy(rx_buffer, cmd_buffer, cmd_len + 1);
					cmd_received = 1;
				}
				cmd_len = 0;
			}
		} else {
			if (cmd_len < 63) {
				cmd_buffer[cmd_len++] = byte;
			}
		}
	}
}

// Callback IDLE line (opcjonalny - dla wykrywania końca transmisji)
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == USART3) {
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
}

/**
 * @brief Ustawia kąt serwa w STOPNIACH (środek = 0°)
 * @param degrees: Kąt w stopniach, zakres -90 do +90
 */
void SetServoAngleDeg(float degrees) {
	// Saturacja do fizycznego zakresu serwa
	if (degrees < -90.0f)
		degrees = -90.0f;
	if (degrees > 90.0f)
		degrees = 90.0f;

	// Przeliczenie: degrees -> PWM (1500 ± degrees/90 * 1000)
	uint32_t pulse_length = (uint32_t) (SERVO_PWM_CENTER + (degrees / SERVO_PWM_RANGE) * 1000.0f);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_length);
}

// Aktualny kąt sprzętowy w stopniach (do slew rate) - NIE static, dostępna z zewnątrz
float g_current_hw_angle_deg = SERVO_CENTER_DEG;

/**
 * @brief Ustawia kąt serwa z slew rate i deadband (w stopniach)
 * @param target_deg: Docelowy kąt w stopniach
 */
void SetServoAngleSmoothDeg(float target_deg) {
	float diff = target_deg - g_current_hw_angle_deg;

	// Deadband - nie aktualizuj przy zbyt małej zmianie
	if (diff > -SERVO_ANGLE_DEADBAND && diff < SERVO_ANGLE_DEADBAND) {
		return;
	}

	// Slew rate limiter
	float max_change = SERVO_SLEW_RATE * (PID_DT_MS / 1000.0f);
	if (diff > max_change) {
		target_deg = g_current_hw_angle_deg + max_change;
	} else if (diff < -max_change) {
		target_deg = g_current_hw_angle_deg - max_change;
	}

	// Saturacja do limitów roboczych
	if (target_deg < SERVO_MIN_DEG)
		target_deg = SERVO_MIN_DEG;
	if (target_deg > SERVO_MAX_DEG)
		target_deg = SERVO_MAX_DEG;

	SetServoAngleDeg(target_deg);
	g_current_hw_angle_deg = target_deg;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	HAL_Delay(500); // Czekaj na ustabilizowanie zasilania czujników
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ETH_Init();
	MX_USART3_UART_Init();
	MX_USB_OTG_FS_PCD_Init();
	MX_TIM3_Init();
	MX_I2C1_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */

	// Initialization moved to StartControlTask
	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of ControlTask */
	osThreadDef(ControlTask, StartControlTask, osPriorityHigh, 0, 1024);
	ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 96;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ETH Initialization Function
 * @param None
 * @retval None
 */
static void MX_ETH_Init(void) {

	/* USER CODE BEGIN ETH_Init 0 */

	/* USER CODE END ETH_Init 0 */

	static uint8_t MACAddr[6];

	/* USER CODE BEGIN ETH_Init 1 */

	/* USER CODE END ETH_Init 1 */
	heth.Instance = ETH;
	MACAddr[0] = 0x00;
	MACAddr[1] = 0x80;
	MACAddr[2] = 0xE1;
	MACAddr[3] = 0x00;
	MACAddr[4] = 0x00;
	MACAddr[5] = 0x00;
	heth.Init.MACAddr = &MACAddr[0];
	heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
	heth.Init.TxDesc = DMATxDscrTab;
	heth.Init.RxDesc = DMARxDscrTab;
	heth.Init.RxBuffLen = 1524;

	/* USER CODE BEGIN MACADDRESS */

	/* USER CODE END MACADDRESS */

	if (HAL_ETH_Init(&heth) != HAL_OK) {
		Error_Handler();
	}

	memset(&TxConfig, 0, sizeof(ETH_TxPacketConfig));
	TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
	TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
	TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
	/* USER CODE BEGIN ETH_Init 2 */

	/* USER CODE END ETH_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x20303E5D;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 95;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 19999;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_PCD_Init(void) {

	/* USER CODE BEGIN USB_OTG_FS_Init 0 */

	/* USER CODE END USB_OTG_FS_Init 0 */

	/* USER CODE BEGIN USB_OTG_FS_Init 1 */

	/* USER CODE END USB_OTG_FS_Init 1 */
	hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
	hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
	hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
	hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
	hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
	hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
	hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
	if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USB_OTG_FS_Init 2 */

	/* USER CODE END USB_OTG_FS_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	/* DMA1_Stream3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, LED_ERR_1_Pin | LED_ERR_2_Pin | LED_ERR_3_Pin | LED_ERR_4_Pin | LED_ERR_5_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : LED_ERR_1_Pin LED_ERR_2_Pin LED_ERR_3_Pin LED_ERR_4_Pin
	 LED_ERR_5_Pin */
	GPIO_InitStruct.Pin = LED_ERR_1_Pin | LED_ERR_2_Pin | LED_ERR_3_Pin | LED_ERR_4_Pin | LED_ERR_5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : USER_Btn_Pin */
	GPIO_InitStruct.Pin = USER_Btn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
	GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : CALIB_START_BTN_Pin CALIB_MID_BTN_Pin CALIB_END_BTN_Pin */
	GPIO_InitStruct.Pin = CALIB_START_BTN_Pin | CALIB_MID_BTN_Pin | CALIB_END_BTN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_OverCurrent_Pin */
	GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument) {
	/* USER CODE BEGIN 5 */
	// Poczekaj na inicjalizację DMA w ControlTask (min 2.2s)
	osDelay(3000);

	HAL_UART_Transmit(&huart3, (uint8_t*) "[DEFAULT] Task started, DMA polling active\r\n", 44, 100);

	/* Infinite loop */
	for (;;) {
		// --- PRZETWARZANIE BUFORA DMA (polling) ---
		// Przetwarzamy tutaj aby komendy (S:, P:, I:, D:) były obsługiwane
		ProcessDmaRxBuffer();

		// Obsługa komend UART (przeniesione z ControlTask)
		if (cmd_received) {
			if (rx_buffer[0] == 'S' && rx_buffer[1] == ':') {
				g_setpoint = atof((char*) &rx_buffer[2]); // S:Setpoint
				if (g_setpoint < 0.0f)
					g_setpoint = 0.0f;
				if (g_setpoint > 250.0f)
					g_setpoint = 250.0f;
			} else if (rx_buffer[0] == 'P' && rx_buffer[1] == ':') {
				g_Kp = atof((char*) &rx_buffer[2]);
			} else if (rx_buffer[0] == 'I' && rx_buffer[1] == ':') {
				g_Ki = atof((char*) &rx_buffer[2]);
			} else if (rx_buffer[0] == 'D' && rx_buffer[1] == ':') {
				g_Kd = atof((char*) &rx_buffer[2]);
			} else if (rx_buffer[0] == 'L' && rx_buffer[1] == '1' && rx_buffer[2] == ':') {
				// LQR K1 gain (pozycja)
				g_K1 = atof((char*) &rx_buffer[3]);
				LQR_UpdateGains(&g_lqr_ctrl, g_K1, g_K2, g_K3);
				sprintf(msg, "LQR K1: %.2f\r\n", g_K1);
				HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 100);
			} else if (rx_buffer[0] == 'L' && rx_buffer[1] == '2' && rx_buffer[2] == ':') {
				// LQR K2 gain (prędkość)
				g_K2 = atof((char*) &rx_buffer[3]);
				LQR_UpdateGains(&g_lqr_ctrl, g_K1, g_K2, g_K3);
				sprintf(msg, "LQR K2: %.2f\r\n", g_K2);
				HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 100);
			} else if (rx_buffer[0] == 'L' && rx_buffer[1] == '3' && rx_buffer[2] == ':') {
				// LQR K3 gain (kąt belki)
				g_K3 = atof((char*) &rx_buffer[3]);
				LQR_UpdateGains(&g_lqr_ctrl, g_K1, g_K2, g_K3);
				sprintf(msg, "LQR K3: %.2f\r\n", g_K3);
				HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 100);
			} else if (rx_buffer[0] == 'X' && rx_buffer[1] == ':') {
				// Przełączanie trybu regulatora: 0 = Custom PID, 1 = LQR
				g_pid_mode = (uint8_t) atoi((char*) &rx_buffer[2]);
				const char *mode_name = (g_pid_mode == 0) ? "CUSTOM PID" : "LQR";
				sprintf(msg, "CONTROLLER MODE: %s\r\n", mode_name);
				HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 100);
			}
			// Komendy Kalibracji
			// Format: "CAL0:50.0,0.0" -> Punkt 0: Surowy=50.0, Rzeczywisty=0.0
			else if (strncmp((char*) rx_buffer, "CAL:RESET", 9) == 0) {
				Calibration_Init();
				sprintf(msg, "[CAL] RESET DONE. Waiting for points...\r\n");
				HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 100);
			} else if (rx_buffer[0] == 'C' && rx_buffer[1] == 'A' && rx_buffer[2] == 'L') {
				int cal_idx = rx_buffer[3] - '0'; // Indeks punktu
				if (cal_idx >= 0 && cal_idx < 5) {
					char *comma = strchr((char*) &rx_buffer[5], ',');
					if (comma != NULL) {
						*comma = '\0';
						float raw_val = atof((char*) &rx_buffer[5]);
						float actual_val = atof(comma + 1);

						// Aktualizacja w bibliotece kalibracyjnej
						Calibration_UpdatePoint(cal_idx, raw_val, actual_val);

						// Sprawdzenie czy mamy komplet punktów
						if (Calibration_IsReady()) {
							sprintf(msg, "[CAL] ✅ All points received! System READY!\r\n");
							HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 100);
						} else {
							// Potwierdzenie przyjęcia punktu
							int r_i = (int) raw_val;
							int r_d = (int) ((raw_val - r_i) * 10);
							int a_i = (int) actual_val;
							int a_d = (int) ((actual_val - a_i) * 10);

							sprintf(msg, "[CAL] Point %d: RAW=%d.%d -> POS=%d.%d (%d/5)\r\n", cal_idx, r_i, abs(r_d),
									a_i, abs(a_d), __builtin_popcount(Calibration_GetReceivedPointsMask()));
							HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 100);
						}
					}
				}
			} else if (rx_buffer[0] == 'M' && rx_buffer[1] == ':') {
				// M:0 -> GUI, M:1 -> Potencjometr, M:2 -> Sinus
				control_mode = (uint8_t) atoi((char*) &rx_buffer[2]);
				const char *mode_names[] = { "GUI", "ANALOG", "SINUS" };
				if (control_mode <= 2) {
					sprintf(msg, "MODE: %s\r\n", mode_names[control_mode]);
					HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 100);
				}
			} else if (rx_buffer[0] == 'R' && rx_buffer[1] == ':') {
				// R:0 -> Stop regulatora, R:1 -> Start regulatora
				g_regulator_enabled = (uint8_t) atoi((char*) &rx_buffer[2]);
				sprintf(msg, "REG: %s\r\n", g_regulator_enabled ? "ON" : "OFF");
				HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 100);

				// Jeśli wyłączony - ustaw serwo na środek
				if (!g_regulator_enabled) {
					// SetServoAngle(SERVO_CENTER); // USUNIĘTE: Teraz używamy g_manual_servo_angle
				}
			} else if (rx_buffer[0] == 'T' && rx_buffer[1] == ':') {
				// T:xx -> Zmiana okresu próbkowania (ms)
				uint32_t new_dt = (uint32_t) atoi((char*) &rx_buffer[2]);
				if (new_dt >= 5 && new_dt <= 500) {
					g_pid_dt_ms = new_dt;
					// g_pid_needs_reinit = 1; // Removed
					sprintf(msg, "DT: %lu ms (%.1f Hz)\r\n", g_pid_dt_ms, 1000.0f / g_pid_dt_ms);
					HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 100);
				} else {
					sprintf(msg, "DT ERR: must be 5-500ms\r\n");
					HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 100);
				}
			} else if (rx_buffer[0] == 'L' && rx_buffer[1] == ':') {
				// L:Angle -> Ustawienie kąta w trybie otwartym (gdy R:0)
				// Teraz akceptujemy kąt w stopniach (-30 do +30)
				g_manual_servo_angle_deg = atof((char*) &rx_buffer[2]);
				if (g_manual_servo_angle_deg < SERVO_MIN_DEG)
					g_manual_servo_angle_deg = SERVO_MIN_DEG;
				if (g_manual_servo_angle_deg > SERVO_MAX_DEG)
					g_manual_servo_angle_deg = SERVO_MAX_DEG;

				// NATYCHMIAST ustaw serwo (bez czekania na główną pętlę)
				SetServoAngleDeg(g_manual_servo_angle_deg);

				// Send ACK for debugging
				sprintf(msg, "L-ACK: %.2f deg\r\n", g_manual_servo_angle_deg);
				HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 10);
			} else if (strncmp((char*) rx_buffer, "TEST:START", 10) == 0) {
				g_test_index = 0;
				g_test_running = 1;
				g_regulator_enabled = 0; // Wyłącz regulator PID
				sprintf(msg, "[TEST] STARTED. Len=%d\r\n", (int) SEQUENCE_LEN);
				HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 100);
			} else if (strncmp((char*) rx_buffer, "TEST:STOP", 9) == 0) {
				g_test_running = 0;
				SetServoAngleDeg(SERVO_CENTER);
				sprintf(msg, "[TEST] STOPPED.\r\n");
				HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 100);
			}

			cmd_received = 0;
		}

		// Aktualizacja LEDów błędu (niekrytyczne czasowo)
		UpdateErrorLEDs_5LED(g_current_error);

		osDelay(5); // 5ms - wystarczająco szybko dla UART
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
 * @brief Function implementing the ControlTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartControlTask */
void StartControlTask(void const *argument) {
	/* USER CODE BEGIN StartControlTask */
	// Uruchomienie PWM dla serwa (Timer 3, Kanał 1)
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

	// Czekamy 2 sekundy
	HAL_UART_Transmit(&huart3, (uint8_t*) "Stabilizing Power...\r\n", 22, 100);
	osDelay(2000);

	HAL_UART_Transmit(&huart3, (uint8_t*) "Setting Servo to CENTER\r\n", 25, 100);
	SetServoAngleDeg(SERVO_CENTER_DEG); // Ustaw serwo na środek (0°)

	HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
	osDelay(200);
	HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);

	HAL_UART_Receive_DMA(&huart3, dma_rx_buffer, DMA_RX_BUFFER_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_TC);

	HAL_UART_Transmit(&huart3, (uint8_t*) "[VISION] DMA UART initialized\r\n", 31, 100);
	HAL_UART_Transmit(&huart3, (uint8_t*) "[VISION] Waiting for Python data...\r\n", 37, 100);
	HAL_UART_Transmit(&huart3, (uint8_t*) "[VISION] Format: V:pos_mm;B:angle_deg;C:CRC\\n\r\n", 47, 100);

	Calibration_Init();

	uint32_t loop_counter = 0;

	LQR_Init(&g_lqr_ctrl, g_K1, g_K2, g_K3, SERVO_MIN_LIMIT, SERVO_MAX_LIMIT);
	ServoPID_Init(&g_servo_pid);

	// Ustaw serwo na pozycję środkową przy starcie (0° = poziomo)
	SetServoAngleDeg(SERVO_CENTER_DEG);

	float prev_valid_dist = 145.0f;
	static float last_servo_angle = SERVO_CENTER_DEG;

	for (int i = 0; i < AVG_ERR_SAMPLES; i++)
		err_buffer[i] = 0.0f;

	/* Infinite loop */
	for (;;) {
		loop_counter++;

		// ProcessDmaRxBuffer wywołany w StartDefaultTask (tam obsługa komend)
		// Tutaj tylko odczytujemy dane wizyjne z volatile zmiennych

		// ServoPID automatycznie pobiera wzmocnienia z g_Kp/g_Ki/g_Kd w każdej iteracji
		// Nie ma potrzeby reinicjalizacji - zmiany są natychmiastowe

		// Obsługa komend UART przeniesiona do StartDefaultTask

		uint8_t vision_timeout = (HAL_GetTick() - g_vision_last_update) > 200;

		float vision_ball_pos;
		float vision_beam_angle;

		if (g_vision_data_valid && !vision_timeout) {
			vision_ball_pos = g_vision_ball_pos;
			vision_beam_angle = g_vision_beam_angle;
			distance = (uint16_t) vision_ball_pos;
		} else {
			vision_ball_pos = prev_valid_dist;
			vision_beam_angle = 0.0f;
			distance = (uint16_t) prev_valid_dist;
		}

		// --- Obsługa Potencjometru ---
		// Wyzwalanie ręczne (Single Mode)
		HAL_ADC_Start(&hadc1); // Start konwersji
		if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
			g_adc_raw = HAL_ADC_GetValue(&hadc1);
			float pot_setpoint = (float) g_adc_raw * 0.070818f;
			g_pot_setpoint = pot_setpoint;

			if (control_mode == 1) {
				static float pot_ema = 0.0f;
				if (pot_ema == 0.0f)
					pot_ema = pot_setpoint;

				pot_ema = 0.1f * pot_setpoint + 0.9f * pot_ema;
				g_setpoint = pot_ema;
			}
		}
		HAL_ADC_Stop(&hadc1);

		if (control_mode == 2) {
			float t = (float) HAL_GetTick() / 1000.0f;
			float period = 5.0f;
			float amplitude = 50.0f;
			float center = 125.0f;

			g_setpoint = center + amplitude * sinf(2.0f * 3.14159f * t / period);
		}

		if (vision_ball_pos >= 0.0f && vision_ball_pos <= 300.0f) {
			prev_valid_dist = vision_ball_pos;
		}

		static float filtered_vision = 125.0f;
		float alpha_vision = 0.7f;
		filtered_vision = alpha_vision * prev_valid_dist + (1.0f - alpha_vision) * filtered_vision;

		uint16_t final_output = (uint16_t) filtered_vision;

		// --- TEST ID SEQUENCE ---
		if (g_test_running) {
			if (g_test_index < SEQUENCE_LEN) {
				uint16_t pwm_val = pwm_sequence[g_test_index];
				// Set PWM directly
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_val);

				// Calculate angle for telemetry (approximation)
				float test_angle = ((float) pwm_val - 500.0f) * 200.0f / 2000.0f;
				last_servo_angle = test_angle;

				// Prepare telemetry variables
				uint16_t dist_display = distance;
				uint16_t filt_display = final_output;

				char data_buffer[96];
				int len = sprintf(data_buffer, "T:%lu;D:%d;Z:%d;A:%d;F:%d;E:0;V:0;B:%d", HAL_GetTick(),
						(int) dist_display, (int) g_setpoint, (int) test_angle, (int) filt_display,
						(int) (vision_beam_angle * 100));  // B: kąt belki * 100 (dla precyzji)
				uint8_t out_crc = CalculateCRC8(data_buffer, len);
				sprintf(msg, "%s;C:%02X\r\n", data_buffer, out_crc);
				HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 10);

				g_test_index++;
			} else {
				// End of sequence
				g_test_running = 0;
				SetServoAngleDeg(SERVO_CENTER_DEG);
				HAL_UART_Transmit(&huart3, (uint8_t*) "TEST:FINISHED\r\n", 15, 100);
			}
			HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
			osDelay(g_pid_dt_ms); // Maintain constant sampling rate!
			continue;
		}

		// --- Sprawdzenie czy regulator jest włączony ---
		if (!g_regulator_enabled) {
			// Regulator wyłączony - sterowanie manualne / otwarte
			SetServoAngleSmoothDeg(g_manual_servo_angle_deg);
			last_servo_angle = g_current_hw_angle_deg;

			// Telemetria - dane z wizji
			uint16_t dist_display = distance;
			uint16_t filt_display = final_output;

			char data_buffer[96];
			int len = sprintf(data_buffer, "T:%lu;D:%d;Z:%d;A:%d;F:%d;E:0;V:0;B:%d", HAL_GetTick(), (int) dist_display,
					(int) g_setpoint, (int) last_servo_angle, (int) filt_display, (int) (vision_beam_angle * 100)); // B: kąt belki * 100
			uint8_t out_crc = CalculateCRC8(data_buffer, len);
			sprintf(msg, "%s;C:%02X\r\n", data_buffer, out_crc);
			HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 10);

			HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
			osDelay(g_pid_dt_ms);
			continue; // Pomiń PID i sterowanie serwem
		}

		// --- Dane z wizji nie wymagają kalibracji czujnika TOF ---
		// Kalibracja OpenCV odbywa się po stronie Pythona

		// filtered_dist = pozycja piłeczki z wizji (już przefiltrowana)
		float filtered_dist = (float) final_output;

		float current_error = g_setpoint - filtered_dist;
		float pid_error = current_error;

		// --- Obliczanie Średniego Błędu (Rolling Average) ---
		float current_abs_err = (current_error < 0) ? -current_error : current_error;

		err_sum -= err_buffer[err_idx];
		err_buffer[err_idx] = current_abs_err;
		err_sum += current_abs_err;

		err_idx++;
		if (err_idx >= AVG_ERR_SAMPLES)
			err_idx = 0;

		float avg_error = err_sum / AVG_ERR_SAMPLES;

		// Strefa nieczułości dla uchybu (stabilizacja w punkcie równowagi)
		if (pid_error > -2.0f && pid_error < 2.0f) {
			pid_error = 0.0f;
		}

		// Warunkowe obliczenie regulatora - wybór implementacji
		float pid_output;
		if (g_pid_mode == 0) {
			float error = g_setpoint - filtered_dist;
			pid_output = ServoPID_Compute(&g_servo_pid, error, filtered_dist);
		} else if (g_pid_mode == 1) {
			float dt = 0.01f;
			float beam_angle_rad = vision_beam_angle * (3.14159f / 180.0f);
			pid_output = LQR_Compute(&g_lqr_ctrl, g_setpoint, filtered_dist, beam_angle_rad, dt);
		} else {
			float error = g_setpoint - filtered_dist;
			pid_output = ServoPID_Compute(&g_servo_pid, error, filtered_dist);
		}
		float pid_angle = pid_output;
		g_current_error = current_error;

		// --- Feedforward dla trybu Sinus ---
		// Analityczna pochodna: d(setpoint)/dt = amplitude * (2π/period) * cos(2π*t/period)
		if (control_mode == 2) {
			float t = (float) HAL_GetTick() / 1000.0f;
			float period = 5.0f;
			float amplitude = 50.0f;
			float omega = 2.0f * 3.14159f / period;  // częstotliwość kątowa

			float setpoint_derivative = amplitude * omega * cosf(omega * t);

			// Kff - współczynnik feedforward (do dostrojenia)
			float Kff = 0.2f;
			float feedforward = Kff * setpoint_derivative;
			pid_angle += feedforward;
		}

		// Rate limiter usunięty - saturacja wystarczy jako zabezpieczenie

		// Zabezpieczenie przed NaN (Not a Number)
		if (isnan(pid_angle) || isinf(pid_angle)) {
			pid_angle = SERVO_CENTER;
		}

		if (pid_angle < SERVO_MIN_LIMIT)
			pid_angle = SERVO_MIN_LIMIT;
		else if (pid_angle > SERVO_MAX_LIMIT)
			pid_angle = SERVO_MAX_LIMIT;

		// Slew Rate Limiter handled by SetServoAngleSmoothDeg
		SetServoAngleSmoothDeg(pid_angle);
		last_servo_angle = g_current_hw_angle_deg;
		volatile float smoothed_angle = last_servo_angle;

		char data_buffer[96];
		// T:Time; D:Dist (raw); Z:Setpoint; A:Angle; F:Filtered; E:Error; V:AvgError; B:BeamAngle*100
		int len = sprintf(data_buffer, "T:%lu;D:%d;Z:%d;A:%d;F:%d;E:%d;V:%d;B:%d", HAL_GetTick(), (int) distance, // D: pozycja piłeczki z wizji (raw)
				(int) g_setpoint,         // Z: setpoint
				(int) smoothed_angle,     // A: kąt serwa
				(int) filtered_dist,      // F: pozycja przefiltrowana
				(int) current_error,      // E: błąd
				(int) avg_error,          // V: średni błąd
				(int) (vision_beam_angle * 100));  // B: kąt belki z wizji * 100

		uint8_t out_crc = CalculateCRC8(data_buffer, len);

		sprintf(msg, "%s;C:%02X\r\n", data_buffer, out_crc);
		static uint8_t u_throttle = 0;
		if (++u_throttle >= 2) {
			u_throttle = 0;
			HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 10); // Timeout 10ms
		}

		osDelay(g_pid_dt_ms); // Loop delay
	}
	/* USER CODE END StartControlTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
