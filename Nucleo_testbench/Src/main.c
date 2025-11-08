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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050_port.h"
#include "qmcr5883l.h"
#include "bmp280.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "gps_parser.h"
#include "mav_messages.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 1024 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for telem_task */
osThreadId_t telem_taskHandle;
uint32_t myTask02Buffer[ 1024 ];
osStaticThreadDef_t myTask02ControlBlock;
const osThreadAttr_t telem_task_attributes = {
  .name = "telem_task",
  .cb_mem = &myTask02ControlBlock,
  .cb_size = sizeof(myTask02ControlBlock),
  .stack_mem = &myTask02Buffer[0],
  .stack_size = sizeof(myTask02Buffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for rc_input */
osThreadId_t rc_inputHandle;
const osThreadAttr_t rc_input_attributes = {
  .name = "rc_input",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for uart_sem */
osSemaphoreId_t uart_semHandle;
const osSemaphoreAttr_t uart_sem_attributes = {
  .name = "uart_sem"
};
/* USER CODE BEGIN PV */

extern uint8_t rxBuffer[128];
extern uint8_t rxIndex;
extern uint8_t rxData;
extern float nmeaLong;
extern float nmeaLat;
extern float utcTime;
extern char posStatus;
extern char northsouth;
extern char eastwest;
extern float decimalLong;
extern float decimalLat;
extern float gpsSpeed;
extern float course;
extern int numSats;
extern float mslAlt;
extern int gpsQuality;
extern int has_fix;
extern int fix_type;
extern float hdop;
extern uint32_t last_led_toggle;
extern char unit;
extern uint32_t gps_send_counter;

volatile uint16_t ppm_live_channels[8];
volatile uint16_t ppm_ready_channels[8];
uint16_t display_channels[8];
volatile uint8_t pulse = 1;

volatile uint8_t ppm_new_data_flag = 0;
volatile uint32_t last_capture = 0;

char msg[100];

volatile uint32_t ICValue_1 = 0;
volatile uint32_t frequency_1 = 0;
volatile float duty_1 = 0;

volatile uint32_t ICValue_2 = 0;
volatile uint32_t frequency_2 = 0;
volatile float duty_2 = 0;

volatile uint32_t ICValue_3 = 0;
volatile uint32_t frequency_3 = 0;
volatile float duty_3 = 0;

volatile uint32_t ICValue_4 = 0;
volatile uint32_t frequency_4 = 0;
volatile float duty_4 = 0;

volatile uint32_t ICValue_5 = 0;
volatile uint32_t frequency_5 = 0;
volatile float duty_5 = 0;

volatile uint32_t ICValue_6 = 0;
volatile uint32_t frequency_6 = 0;
volatile float duty_6 = 0;

volatile float pulse_width_us_1;
volatile float pulse_width_us_2;
volatile float pulse_width_us_3;
volatile float pulse_width_us_4;
volatile float pulse_width_us_5;
volatile float pulse_width_us_6;

volatile float batt_value = 0;
volatile float adc_value = 0;
volatile float internal_temp = 0;
volatile float v_out = 0;

int disarm_flag;
int arm_flag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
void servo_mapping(void *argument);
void telemetry_task(void *argument);
void arm_disarm(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);

	return ch;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		uint32_t current_capture = HAL_TIM_ReadCapturedValue(htim,
		TIM_CHANNEL_1);
		uint16_t pulse_width;

		if (current_capture > last_capture) {
			pulse_width = current_capture - last_capture;
		} else {
			pulse_width = (65535 - last_capture) + current_capture;
		}

		if (pulse_width > 3000) {

			memcpy((void*) ppm_ready_channels, (void*) ppm_live_channels,
					sizeof(ppm_live_channels));
			ppm_new_data_flag = 1;

			pulse = 1;
		}

		else {
			switch (pulse) {
			case 1:
				ppm_live_channels[0] = pulse_width;
				pulse++;
				break;
			case 2:
				ppm_live_channels[1] = pulse_width;
				pulse++;
				break;
			case 3:
				ppm_live_channels[2] = pulse_width;
				pulse++;
				break;
			case 4:
				ppm_live_channels[3] = pulse_width;
				pulse++;
				break;
			case 5:
				ppm_live_channels[4] = pulse_width;
				pulse++;
				break;
			case 6:
				ppm_live_channels[5] = pulse_width;
				pulse++;
				break;
			default:

				pulse++;
				break;
			}
		}
		last_capture = current_capture;
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {

		if (rxData != '\n' && rxIndex < sizeof(rxBuffer)) {
			rxBuffer[rxIndex++] = rxData;
		} else {
			if (gpsValidate((char*) rxBuffer))
				gpsParse((char*) rxBuffer);
			rxIndex = 0;
			memset(rxBuffer, 0, sizeof(rxBuffer));
		}
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		HAL_UART_Receive_IT(&huart1, &rxData, 1);

	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	adc_value = HAL_ADC_GetValue(&hadc1);
	v_out = (adc_value * 3.3f) / 4096;
	batt_value = v_out / (7500.0f / (30000.0f + 7500.0f));
	//printf("Itr triggered\n");
	HAL_ADC_Start_IT(&hadc1);

}

uint32_t mapRCtoMotor(uint16_t rcValue) {
	if (rcValue > 2000) {
		rcValue = 2000;
	}
	if (rcValue < 1000) {
		rcValue = 1000;
	}
	// used simple ratio vala logic (y-0/63000-0 = x-1000 / 2000-1000)
	uint32_t motorValue = (uint32_t) (rcValue - 1000) * 180 / 1000;
	return motorValue;
}

void setServoAngle(uint32_t angle, int channel) {
	if (angle > 180)
		angle = 180;

	uint32_t minPulseWidth = 500;   // 0.5 ms
	uint32_t maxPulseWidth = 2500;  // 2.5 ms
	uint32_t pulse = ((angle * (maxPulseWidth - minPulseWidth)) / 180)
			+ minPulseWidth;

	if (channel == 1) {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
	} else if (channel == 2) {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
	} else if (channel == 3) {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse);
	} else if (channel == 4) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
	} else if (channel == 5) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);
	} else if (channel == 6) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse);
	} else if (channel == 7) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulse);
	}

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of uart_sem */
  uart_semHandle = osSemaphoreNew(1, 1, &uart_sem_attributes);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(servo_mapping, NULL, &defaultTask_attributes);

  /* creation of telem_task */
  telem_taskHandle = osThreadNew(telemetry_task, NULL, &telem_task_attributes);

  /* creation of rc_input */
  rc_inputHandle = osThreadNew(arm_disarm, NULL, &rc_input_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 5;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_servo_mapping */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_servo_mapping */
void servo_mapping(void *argument)
{
  /* USER CODE BEGIN 5 */

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //pwm out tim2
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //pwm out tim3
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	/* Infinite loop */
	for (;;) {

		if (ppm_new_data_flag) {

			memcpy(display_channels, (void*) ppm_ready_channels,
					sizeof(ppm_ready_channels));
			ppm_new_data_flag = 0;


			if (arm_flag == 1 && disarm_flag == 0)
			{
				//HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				//	HAL_MAX_DELAY);

				//for debug
				/*printf(
				 "Normalized: 0: %lu, Normal: %u | 1: %lu, Normal: %u | 2: %lu, Normal: %u | 3: %lu, Normal: %u | 4: %lu, Normal: %u | 5: %lu, Normal: %u\n",
				 mapRCtoMotor(display_channels[0]), display_channels[0],
				 mapRCtoMotor(display_channels[1]), display_channels[1],
				 mapRCtoMotor(display_channels[2]), display_channels[2],
				 mapRCtoMotor(display_channels[3]), display_channels[3],
				 mapRCtoMotor(display_channels[4]), display_channels[4],
				 mapRCtoMotor(display_channels[5]), display_channels[5]);
				 */

				setServoAngle(mapRCtoMotor(display_channels[0]), 1);
				setServoAngle(mapRCtoMotor(display_channels[1]), 2);
				setServoAngle(mapRCtoMotor(display_channels[2]), 3);
				setServoAngle(mapRCtoMotor(display_channels[3]), 4);
				setServoAngle(mapRCtoMotor(display_channels[4]), 5);
				setServoAngle(mapRCtoMotor(display_channels[5]), 6);


			}
			else if (arm_flag == 0 && disarm_flag == 1)
					{
						//HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
						//	HAL_MAX_DELAY);

						//for debug
						/*printf(
						 "Normalized: 0: %lu, Normal: %u | 1: %lu, Normal: %u | 2: %lu, Normal: %u | 3: %lu, Normal: %u | 4: %lu, Normal: %u | 5: %lu, Normal: %u\n",
						 mapRCtoMotor(display_channels[0]), display_channels[0],
						 mapRCtoMotor(display_channels[1]), display_channels[1],
						 mapRCtoMotor(display_channels[2]), display_channels[2],
						 mapRCtoMotor(display_channels[3]), display_channels[3],
						 mapRCtoMotor(display_channels[4]), display_channels[4],
						 mapRCtoMotor(display_channels[5]), display_channels[5]);
						 */

						setServoAngle(0, 1);
						setServoAngle(0, 2);
						setServoAngle(0, 3);
						setServoAngle(0, 4);
						setServoAngle(0, 5);
						setServoAngle(0, 6);


					}




		}

	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_telemetry_task */
/**
 * @brief Function implementing the myTask02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_telemetry_task */
void telemetry_task(void *argument)
{
  /* USER CODE BEGIN telemetry_task */
	/* Infinite loop */
	HAL_UART_Receive_IT(&huart1, &rxData, 1);
	mpu_init();
	printf("Init ok mpu lol/n");
	qmc_init();
	printf("Init ok qmc/n");
	bmp_i2c_setup();
	printf("Init ok bmp/n");

	//HAL_ADC_Start_IT(&hadc1);

	for (;;) {
		osDelay(20);

		if (arm_flag == 1 && disarm_flag == 0) {

			send_heartbeat_armed();
			send_attitude();
			send_heartbeat_armed();
			send_global_position_int();
			send_heartbeat_armed();
			send_battery_info();
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);

			//osSemaphoreRelease(arm_semHandle);
		} else if (arm_flag == 0 && disarm_flag == 1) {

			send_heartbeat_disarmed();
			send_attitude();
			send_heartbeat_disarmed();
			send_global_position_int();
			send_heartbeat_disarmed();
			send_battery_info();
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);

		}
	}

  /* USER CODE END telemetry_task */
}

/* USER CODE BEGIN Header_arm_disarm */
	/**
	 * @brief Function implementing the rc_input thread.
	 * @param argument: Not used
	 * @retval None
	 */
/* USER CODE END Header_arm_disarm */
void arm_disarm(void *argument)
{
  /* USER CODE BEGIN arm_disarm */
		HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
		/* Infinite loop */
		for (;;) {
			/*
			printf(

			 "Normalized: 0: %lu, Normal: %u | 1: %lu, Normal: %u | 2: %lu, Normal: %u | 3: %lu, Normal: %u | 4: %lu, Normal: %u | 5: %lu, Normal: %u\n",
			 mapRCtoMotor(display_channels[0]), display_channels[0],
			 mapRCtoMotor(display_channels[1]), display_channels[1],
			 mapRCtoMotor(display_channels[2]), display_channels[2],
			 mapRCtoMotor(display_channels[3]), display_channels[3],
			 mapRCtoMotor(display_channels[4]), display_channels[4],
			 mapRCtoMotor(display_channels[5]), display_channels[5]);
			 */

			if (display_channels[4] > 1900) {
				arm_flag = 1;
				disarm_flag = 0;
			} else if (display_channels[4]< 1900) {
				arm_flag = 0;
				disarm_flag = 1;
			}
			osDelay(1);
		}
  /* USER CODE END arm_disarm */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
		/* User can add his own implementation to report the HAL error return state */
		__disable_irq();
		while (1) {
		}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
