/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body for interrupt-based NEO-6M GPS parsing with full features (time, pos, speed, course, sats, alt, fix)
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>  // For potential conversions
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Enhanced variables for GPS parsing (added course, sats, alt, quality)
uint8_t rxBuffer[128] = {0};
uint8_t rxIndex = 0;
uint8_t rxData;
float nmeaLong;
float nmeaLat;
float utcTime;
char northsouth;
char eastwest;
char posStatus;
float decimalLong;
float decimalLat;
float gpsSpeed;  // Ground speed in knots (GPRMC field 7)
float course;    // Course over ground in degrees (GPRMC field 8)
int numSats = 0; // Number of satellites used (GPGGA field 7)
float mslAlt = 0.0f; // MSL altitude in meters (GPGGA field 9)
int gpsQuality = 0;  // Fix quality (GPGGA field 6: 0=no fix, 1=GPS fix, etc.)
int has_fix = 0;     // Overall fix status flag (0: no, 1: yes)
uint32_t last_led_toggle = 0;  // For LED throttling
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
// Enhanced prototypes
float nmeaToDecimal(float coordinate);
void gpsParse(char *strParse);
int gpsValidate(char *nmea);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PUTCHAR_PROTOTYPE {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

// nmeaToDecimal function (unchanged)
float nmeaToDecimal(float coordinate) {
    int degree = (int)(coordinate/100);
    float minutes = coordinate - degree * 100;
    float decimalDegree = minutes / 60;
    float decimal = degree + decimalDegree;
    return decimal;
}

// gpsValidate function (unchanged)
int gpsValidate(char *nmea){
    char check[3];
    char calculatedString[3];
    int index;
    int calculatedCheck;

    index=0;
    calculatedCheck=0;

    // Ensure that the string starts with a "$"
    if(nmea[index] == '$')
        index++;
    else
        return 0;

    //No NULL reached, 75 char largest possible NMEA message, no '*' reached
    while((nmea[index] != 0) && (nmea[index] != '*') && (index < 75)){
        calculatedCheck ^= nmea[index];// calculate the checksum
        index++;
    }

    if(index >= 75){
        return 0;// the string is too long so return an error
    }

    if (nmea[index] == '*'){
        check[0] = nmea[index+1];    //put hex chars in check string
        check[1] = nmea[index+2];
        check[2] = 0;
    }
    else
        return 0;// no checksum separator found therefore invalid data

    sprintf(calculatedString,"%02X",calculatedCheck);
    return((calculatedString[0] == check[0])
        && (calculatedString[1] == check[1])) ? 1 : 0 ;
}

// Enhanced gpsParse function (added course from RMC, sats/alt/quality from GGA; update fix from both)
void gpsParse(char *strParse){
    int new_fix_rmc = 0;
    int new_fix_gga = 0;

    if(!strncmp(strParse, "$GPGGA", 6)){
        // Extended sscanf for GGA: time, lat, NS, lon, EW, quality, sats, HDOP (skip), alt, units
        char alt_units;
        sscanf(strParse, "$GPGGA,%f,%f,%c,%f,%c,%d,%d,%*f,%f,%c",
               &utcTime, &nmeaLat, &northsouth, &nmeaLong, &eastwest,
               &gpsQuality, &numSats, &mslAlt, &alt_units);
        decimalLat = nmeaToDecimal(nmeaLat);
        decimalLong = nmeaToDecimal(nmeaLong);
        new_fix_gga = (gpsQuality > 0) ? 1 : 0;  // Fix if quality >=1
    }
    else if (!strncmp(strParse, "$GPGLL", 6)){
        sscanf(strParse, "$GPGLL,%f,%c,%f,%c,%f",
               &nmeaLat, &northsouth, &nmeaLong, &eastwest, &utcTime);
        decimalLat = nmeaToDecimal(nmeaLat);
        decimalLong = nmeaToDecimal(nmeaLong);
    }
    else if (!strncmp(strParse, "$GPRMC", 6)){
        // Fixed sscanf for RMC: time, status, lat, NS, lon, EW, speed (field7), course (field8), skip date
        sscanf(strParse, "$GPRMC,%f,%c,%f,%c,%f,%c,%f,%f,%*s",
               &utcTime, &posStatus, &nmeaLat, &northsouth, &nmeaLong, &eastwest, &gpsSpeed, &course);
        decimalLat = nmeaToDecimal(nmeaLat);
        decimalLong = nmeaToDecimal(nmeaLong);
        new_fix_rmc = (posStatus == 'A') ? 1 : 0;  // Active fix
    }

    // Update overall fix status (1 if either RMC 'A' or GGA quality >0)
    int overall_fix = (new_fix_rmc || new_fix_gga) ? 1 : 0;
    if (overall_fix != has_fix) {
        has_fix = overall_fix;
        if (has_fix) {
            printf("GPS Fix Acquired! (RMC: %c, GGA Quality: %d)\n", posStatus, gpsQuality);
        } else {
            printf("No GPS Fix (RMC: %c, GGA Quality: %d) - Searching...\n", posStatus, gpsQuality);
        }
    }

    // Print all data only on valid fix and RMC (using latest GGA sats/alt)
    if (has_fix && strncmp(strParse, "$GPRMC", 6) == 0) {
        // Format time (hhmmss.ss -> hh:mm:ss)
        char time_str[9];
        snprintf(time_str, sizeof(time_str), "%.6f", utcTime);
        time_str[2] = ':';
        time_str[5] = ':';
        time_str[8] = '\0';

        // Adjust signs for lat/lon
        if (northsouth == 'S') decimalLat = -decimalLat;
        if (eastwest == 'W') decimalLong = -decimalLong;

        // Convert speed to km/h
        float speedKmh = gpsSpeed * 1.852;

        printf("Fix Active - Time: %s UTC\n", time_str);
        printf("Lat: %.6f° (%c), Lon: %.6f° (%c)\n", decimalLat, northsouth, decimalLong, eastwest);
        printf("Ground Speed: %.2f knots (%.2f km/h)\n", gpsSpeed, speedKmh);
        printf("Course over Ground: %.1f° (true)\n", course);
        printf("Satellites Used: %d, MSL Altitude: %.2f m\n", numSats, mslAlt);
        printf("--------------------------------\n");
    }
}

// HAL_UART_RxCpltCallback (minor adjustment: ensure \r is handled, but process on \n)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==USART1)
  {
    // Add char if not \n and buffer not full (NMEA ends with \r\n, so \r will be added, \n triggers)
    if(rxData != '\n' && rxIndex < sizeof(rxBuffer) - 1)
    {
      rxBuffer[rxIndex++] = rxData;
    }
    else
    {
      rxBuffer[rxIndex] = '\0';  // Null-terminate
      if(gpsValidate((char*) rxBuffer)) gpsParse((char*) rxBuffer);
      rxIndex = 0;
      memset(rxBuffer, 0, sizeof(rxBuffer));
    }
    HAL_UART_Receive_IT(&huart1, &rxData, 1); // Re-enable interrupt
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("STM32 NEO-6M GPS Enhanced Parser Started (Time, Pos, Speed, Course, Sats, Alt, Fix). Waiting for data...\n");
  HAL_UART_Receive_IT(&huart1, &rxData, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // LED control based on fix status (throttled)
    uint32_t now = HAL_GetTick();
    if (has_fix) {
        if (now - last_led_toggle >= 500) {  // Fast blink for fix
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            last_led_toggle = now;
        }
    } else {
        if (now - last_led_toggle >= 2000) {  // Slow blink for no fix
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            last_led_toggle = now;
        }
    }
    HAL_Delay(10);
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief USART1 Initialization Function (GPS input, 9600 baud, interrupt-enabled)
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  // Enable UART1 global interrupt
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
 * @brief USART2 Initialization Function (serial monitor, 115200 baud)
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{
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
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin (status LED) */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
