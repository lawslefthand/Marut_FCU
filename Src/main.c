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
#include "mavlink/common/mavlink.h"
#include "mpu6050_port.h"
#include "qmcr5883l.h"
#include "bmp280.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Telemetry */
osThreadId_t TelemetryHandle;
uint32_t TelemetryBuffer[ 1024 ];
osStaticThreadDef_t TelemetryControlBlock;
const osThreadAttr_t Telemetry_attributes = {
  .name = "Telemetry",
  .cb_mem = &TelemetryControlBlock,
  .cb_size = sizeof(TelemetryControlBlock),
  .stack_mem = &TelemetryBuffer[0],
  .stack_size = sizeof(TelemetryBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
uint8_t rxBuffer[128] = { 0 };
uint8_t rxIndex = 0;
uint8_t rxData;
float nmeaLong;
float nmeaLat;
float utcTime;
char posStatus;
char northsouth;
char eastwest;
float decimalLong;
float decimalLat;
float gpsSpeed;
float course;
int numSats = 0;
float mslAlt = 0.0f;
int gpsQuality = 0;
int has_fix = 0;
int fix_type = 0;
float hdop = 0.0f;
uint32_t last_led_toggle = 0;
char unit;
uint32_t gps_send_counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void mav_telem(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float nmeaToDecimal(float coordinate) {
	int degree = (int) (coordinate / 100);
	float minutes = coordinate - degree * 100;
	float decimalDegree = minutes / 60;
	float decimal = degree + decimalDegree;
	return decimal;
}

int gpsValidate(char *nmea) {
	char check[3];
	char calculatedString[3];
	int index;
	int calculatedCheck;

	index = 0;
	calculatedCheck = 0;

	if (nmea[index] == '$')
		index++;
	else
		return 0;

	while ((nmea[index] != 0) && (nmea[index] != '*') && (index < 75)) {
		calculatedCheck ^= nmea[index];
		index++;
	}

	if (index >= 75) {
		return 0;
	}

	if (nmea[index] == '*') {
		check[0] = nmea[index + 1];
		check[1] = nmea[index + 2];
		check[2] = 0;
	} else
		return 0;

	sprintf(calculatedString, "%02X", calculatedCheck);
	return ((calculatedString[0] == check[0])
			&& (calculatedString[1] == check[1])) ? 1 : 0;
}

void gpsParse(char *strParse) {
	int new_fix = 0;
	if (!strncmp(strParse, "$GPGGA", 6)) {

		sscanf(strParse, "$GPGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c", &utcTime,
				&nmeaLat, &northsouth, &nmeaLong, &eastwest, &gpsQuality,
				&numSats, &hdop, &mslAlt, &unit);
		decimalLat = nmeaToDecimal(nmeaLat);
		decimalLong = nmeaToDecimal(nmeaLong);

		if (gpsQuality > 0) {
			switch (gpsQuality) {
			case 0:
				fix_type = 0;
				break;
			case 1:
				fix_type = 3;
				break;
			case 2:
				fix_type = 4;
				break;
			default:
				fix_type = 3;
				break;
			}
			has_fix = 1;
			new_fix = 1;
			printf("GPS Fix Acquired from GGA! Quality: %d, Satellites: %d\n",
					gpsQuality, numSats);
		} else {
			fix_type = 0;
			has_fix = 0;
		}
	} else if (!strncmp(strParse, "$GPGLL", 6)) {
		sscanf(strParse, "$GPGLL,%f,%c,%f,%c,%f", &nmeaLat, &northsouth,
				&nmeaLong, &eastwest, &utcTime);
		decimalLat = nmeaToDecimal(nmeaLat);
		decimalLong = nmeaToDecimal(nmeaLong);
	} else if (!strncmp(strParse, "$GPRMC", 6)) {

		sscanf(strParse, "$GPRMC,%f,%c,%f,%c,%f,%c,%f,%f,%*s", &utcTime,
				&posStatus, &nmeaLat, &northsouth, &nmeaLong, &eastwest,
				&gpsSpeed, &course);
		decimalLat = nmeaToDecimal(nmeaLat);
		decimalLong = nmeaToDecimal(nmeaLong);
		new_fix = (posStatus == 'A') ? 1 : 0;
		if (new_fix != has_fix) {
			has_fix = new_fix;
			fix_type = new_fix ? 3 : 0;
			if (has_fix) {
				printf("GPS Fix Active! (Status: A)\n");
			} else {
				printf("No GPS Fix (Status: V) - Searching...\n");
			}
		}
	}

	if (has_fix && strncmp(strParse, "$GPRMC", 6) == 0) {

		float signedLat = (northsouth == 'S') ? -decimalLat : decimalLat;
		float signedLon = (eastwest == 'W') ? -decimalLong : decimalLong;
		printf(
				"Fix Active - Lat: %.6f, Lon: %.6f, Speed: %.2f knots, Course: %.1f°, Sats: %d, Alt: %.2f m\n",
				signedLat, signedLon, gpsSpeed, course, numSats, mslAlt);
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
		HAL_UART_Receive_IT(&huart1, &rxData, 1);
	}
}

void send_heartbeat(void) {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_GENERIC,
			MAV_AUTOPILOT_GENERIC, MAV_MODE_MANUAL_ARMED, 0, MAV_STATE_ACTIVE);

	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

	HAL_UART_Transmit(&huart2, buf, len, HAL_MAX_DELAY);
}

void send_attitude(void) {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;

	uint8_t my_system_id = 1;
	uint8_t my_component_id = 200;

	mavlink_msg_attitude_pack(my_system_id, my_component_id, &msg,
			HAL_GetTick(), mpu_accel_read(0) * (3.14 / 180),
			mpu_accel_read(1) * (3.14 / 180), qmc_mag_read() * (3.14 / 180),
			3.0f, 3.0f, 3.0f);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	HAL_UART_Transmit(&huart2, buf, len, HAL_MAX_DELAY);
}

void send_gps_raw_int(void) {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;

	uint8_t my_system_id = 1;
	uint8_t my_component_id = 200;

	int32_t lat_int = (int32_t) (decimalLat * 1E7);
	int32_t lon_int = (int32_t) (decimalLong * 1E7);

	int32_t alt_mm = (int32_t) (mslAlt * 1000.0f);

	float speed_ms = gpsSpeed * 0.514444f;
	uint16_t vel_cm_s = (uint16_t) (speed_ms * 100.0f);
	uint16_t cog_cdeg = (uint16_t) (course * 100.0f);
	uint16_t eph = (hdop > 0.0f) ? (uint16_t) (hdop * 100.0f) : UINT16_MAX;
	uint16_t epv = UINT16_MAX;

	mavlink_msg_gps_raw_int_pack(my_system_id, my_component_id, &msg,
			(uint64_t) HAL_GetTick() * 1000ULL, fix_type, lat_int, lon_int,

			alt_mm,

			eph, epv, vel_cm_s, cog_cdeg, (uint8_t) numSats,

			alt_mm,

			UINT32_MAX,
			UINT32_MAX,
			UINT32_MAX,
			UINT32_MAX, 0);

	len = mavlink_msg_to_send_buffer(buf, &msg);
	HAL_UART_Transmit(&huart2, buf, len, HAL_MAX_DELAY);
	printf("Sent GPS MAVLink: Fix %d, Lat %.6f, Lon %.6f, Sats %d\n", fix_type,
			decimalLat, decimalLong, numSats);
}

void send_global_position_int(void) {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	uint8_t my_system_id = 1;
	uint8_t my_component_id = 200;
	uint32_t time_boot_ms = HAL_GetTick();

	float relative_alt_meters = altitude_calc();

	int32_t lat = (int32_t) (decimalLat * 1E7);
	int32_t lon = (int32_t) (decimalLong * 1E7);

	int32_t alt = (int32_t) (mslAlt * 1000.0f);

	int32_t relative_alt_mm = (int32_t) (relative_alt_meters * 1000.0f);

	int16_t vx = 0;
	int16_t vy = 0;
	int16_t vz = 0;

	uint16_t hdg = (uint16_t) (course * 100.0f);

	mavlink_msg_global_position_int_pack(my_system_id, my_component_id, &msg,
			time_boot_ms, lat, lon, alt, relative_alt_mm, vx, vy, vz, hdg);

	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	HAL_UART_Transmit(&huart2, buf, len, HAL_MAX_DELAY);
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Telemetry */
  TelemetryHandle = osThreadNew(mav_telem, NULL, &Telemetry_attributes);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart1.Init.BaudRate = 115200;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {

		osDelay(1);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_mav_telem */
/**
 * @brief Function implementing the Telemetry thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_mav_telem */
void mav_telem(void *argument)
{
  /* USER CODE BEGIN mav_telem */
	mpu_init();  // Unchanged sensor init
	qmc_init();  // Unchanged sensor init
	bmp_i2c_setup();
	//HAL_UART_Receive_IT(&huart1, &rxData, 1);  // Start GPS interrupt reception

	/* Infinite loop */
	for (;;) {
		send_heartbeat();
		send_attitude();
		send_global_position_int();
		send_heartbeat();
		osDelay(1);
	}
  /* USER CODE END mav_telem */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
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
