/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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

#include "mpu6050.h"
#include "bmp280.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "mav_messages.h"
#include "mavlink/common/mavlink.h"
#include  <stdint.h>
#include "Support_function.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

#define EL_L_DIR   (+1)
#define EL_R_DIR   (-1)
#define AILERON_LEFT_CH   1
#define AILERON_RIGHT_CH  2

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* Definitions for ArmDisarm */
osThreadId_t ArmDisarmHandle;
uint32_t ArmDisarmBuffer[128];
osStaticThreadDef_t ArmDisarmControlBlock;
const osThreadAttr_t ArmDisarm_attributes = { .name = "ArmDisarm", .cb_mem =
		&ArmDisarmControlBlock, .cb_size = sizeof(ArmDisarmControlBlock),
		.stack_mem = &ArmDisarmBuffer[0], .stack_size = sizeof(ArmDisarmBuffer),
		.priority = (osPriority_t) osPriorityLow, };
/* Definitions for ModeHandler */
osThreadId_t ModeHandlerHandle;
uint32_t ModeHandlerBuffer[128];
osStaticThreadDef_t ModeHandlerControlBlock;
const osThreadAttr_t ModeHandler_attributes = { .name = "ModeHandler", .cb_mem =
		&ModeHandlerControlBlock, .cb_size = sizeof(ModeHandlerControlBlock),
		.stack_mem = &ModeHandlerBuffer[0], .stack_size =
				sizeof(ModeHandlerBuffer), .priority =
				(osPriority_t) osPriorityLow, };
/* Definitions for Debounce_Handle */
osThreadId_t Debounce_HandleHandle;
uint32_t Debounce_HandleBuffer[128];
osStaticThreadDef_t Debounce_HandleControlBlock;
const osThreadAttr_t Debounce_Handle_attributes = { .name = "Debounce_Handle",
		.cb_mem = &Debounce_HandleControlBlock, .cb_size =
				sizeof(Debounce_HandleControlBlock), .stack_mem =
				&Debounce_HandleBuffer[0], .stack_size =
				sizeof(Debounce_HandleBuffer), .priority =
				(osPriority_t) osPriorityLow, };
/* Definitions for QuadTask */
osThreadId_t QuadTaskHandle;
uint32_t QuadTaskBuffer[2024];
osStaticThreadDef_t QuadTaskControlBlock;
const osThreadAttr_t QuadTask_attributes = { .name = "QuadTask", .cb_mem =
		&QuadTaskControlBlock, .cb_size = sizeof(QuadTaskControlBlock),
		.stack_mem = &QuadTaskBuffer[0], .stack_size = sizeof(QuadTaskBuffer),
		.priority = (osPriority_t) osPriorityRealtime, };
/* Definitions for TelemetryTask */
osThreadId_t TelemetryTaskHandle;
uint32_t TelemetryTaskBuffer[2024];
osStaticThreadDef_t TelemetryTaskControlBlock;
const osThreadAttr_t TelemetryTask_attributes = { .name = "TelemetryTask",
		.cb_mem = &TelemetryTaskControlBlock, .cb_size =
				sizeof(TelemetryTaskControlBlock), .stack_mem =
				&TelemetryTaskBuffer[0], .stack_size =
				sizeof(TelemetryTaskBuffer), .priority =
				(osPriority_t) osPriorityNormal, };
/* Definitions for FixedWingTask */
osThreadId_t FixedWingTaskHandle;
uint32_t FixedWingTaskBuffer[2024];
osStaticThreadDef_t FixedWingTaskControlBlock;
const osThreadAttr_t FixedWingTask_attributes = { .name = "FixedWingTask",
		.cb_mem = &FixedWingTaskControlBlock, .cb_size =
				sizeof(FixedWingTaskControlBlock), .stack_mem =
				&FixedWingTaskBuffer[0], .stack_size =
				sizeof(FixedWingTaskBuffer), .priority =
				(osPriority_t) osPriorityRealtime, };
/* Definitions for VtolMode */
osThreadId_t VtolModeHandle;
uint32_t VtolModeBuffer[2024];
osStaticThreadDef_t VtolModeControlBlock;
const osThreadAttr_t VtolMode_attributes = { .name = "VtolMode", .cb_mem =
		&VtolModeControlBlock, .cb_size = sizeof(VtolModeControlBlock),
		.stack_mem = &VtolModeBuffer[0], .stack_size = sizeof(VtolModeBuffer),
		.priority = (osPriority_t) osPriorityRealtime, };
/* Definitions for timer_sem */
osSemaphoreId_t timer_semHandle;
const osSemaphoreAttr_t timer_sem_attributes = { .name = "timer_sem" };
/* USER CODE BEGIN PV */

volatile uint16_t ppm_live_channels[6];
volatile uint16_t ppm_ready_channels[6];

uint16_t display_channels[6]; // NOTE : Keep Channels uniform
volatile uint8_t pulse = 1;

volatile uint8_t ppm_new_data_flag = 0;
volatile uint32_t last_capture = 0;

char msg[100];

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

float dt = 0;

static int disarm_flag;
static int arm_flag;
int button_counter;
static int mode_flag = 0;
static int temp_mode_flag = 0;

int elevon_left = 0;
int elevon_right = 0;

float calibration_const_global_roll_accel = 0;
float calibration_const_global_pitch_accel = 0;

float calibration_const_global_roll_gyro = 0;
float calibration_const_global_pitch_gyro = 0;

float calibration_const_global_gx = 0;
float calibration_const_global_gy = 0;
float calibration_const_global_gz = 0;

float calibration_const_global_ax = 0;
float calibration_const_global_ay = 0;
float calibration_const_global_az = 0;

float kalman_roll = 0.0f;
float kalman_pitch = 0.0f;
float roll_snap = 0.0f;
float pitch_snap = 0.0f;
int32_t altitude_snap = 0.0f;

float desired_rate_roll;
float desired_rate_pitch;
float desired_rate_yaw;

float error_rate_roll;
float error_rate_pitch;
float error_rate_yaw;

float input_roll;
float input_throttle;
float input_pitch;
float input_yaw;

float prev_error_rate_roll;
float prev_error_rate_pitch;
float prev_error_rate_yaw;

float prev_iterm_rate_roll;
float prev_iterm_rate_pitch;
float prev_iterm_rate_yaw;

float PIDReturn[] = { 0, 0, 0 };

float desired_angle_roll;
float desired_angle_pitch;

float error_angle_roll = 0;
float error_angle_pitch = 0;

float prev_error_angle_roll = 0;
float prev_error_angle_pitch = 0;

float prev_iterm_angle_roll = 0;
float prev_iterm_angle_pitch = 0;

int quad_mode_flag = 0;

float acc_z_inertial = 0.0f;

float velocity_vertical = 0.0f;

uint32_t last_time;

uint32_t M1 = 0;
uint32_t M2 = 0;
uint32_t M3 = 0;
uint32_t M4 = 0;

float gyro_x = 0;
float gyro_y = 0;
float gyro_z = 0;

float accel_roll = 0;
float accel_pitch = 0;

float relative_alt_meters = 0.0f;
float qmc_mag_ext = 0.0f;

float gx;
float gy;
float gz;

// Note: PID Globals

// RATE LOOP
float p_rate_roll = 0.500f; // 0.47000 absolute limit, these are stable values for 5 inch quad (qav200)
float p_rate_pitch = 0.500f;
float p_rate_yaw = 2.50f;

float i_rate_roll = 0.100f;
float i_rate_pitch = 0.100f;
float i_rate_yaw = 0.80f;

float d_rate_roll = 0.010f;
float d_rate_pitch = 0.010f;
float d_rate_yaw = 0.0f;

// ANGLE LOOP
float p_angle_roll = 0.0f;
float p_angle_pitch = 0.0f;

float i_angle_roll = 0.0f;
float i_angle_pitch = 0.0f;

float d_angle_roll = 0.0f;
float d_angle_pitch = 0.0f;

float xx = 0;
float yy = 0;

uint32_t current_tick_quad = 0;
uint32_t current_tick_fw = 0;
uint32_t current_tick_vtol = 0;

//Baro and Mag external read variables
float alt_ext = 0;
float mag_ext = 0;

// Status variables
int mode_task = 0;
int quad_task = 0;
int bounce_task = 0;
int telemetery_task = 0;
int arm_disarm_task = 0;
int itr_running = 100;
uint32_t t1;
uint32_t t2;
uint32_t dt_n;
int tim_flag = 0;

int in_stabilize = 0;
int in_rate = 0;

//SW Timer variables
static uint32_t last = 0;
uint32_t now;

int m_t = 0;
int measure = 0;

float g_global_x = 0;
float g_global_y = 0;
float g_global_z = 0;

float a_global_x = 0;
float a_global_y = 0;
float a_global_z = 0;

//RC Calibration (preliminary) variables

extern int rc_max_us;
extern int rc_min_us;

//RC Channel debug variables

volatile uint16_t ch1_dbg;
volatile uint16_t ch2_dbg;
volatile uint16_t ch3_dbg;
volatile uint16_t ch4_dbg;
volatile uint16_t ch5_dbg;
volatile uint16_t ch6_dbg;
volatile uint16_t ch7_dbg;
volatile uint16_t ch8_dbg;
volatile uint16_t ch9_dbg;
volatile uint16_t ch10_dbg;

uint16_t deadband_filter_temp_ch1 = 0;
uint16_t deadband_filter_temp_ch2 = 0;
uint16_t deadband_filter_temp_ch3 = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
void arm_disarm(void *argument);
void mode_handler(void *argument);
void debounce_task(void *argument);
void quad_mode(void *argument);
void telemetry_task(void *argument);
void fw_mode(void *argument);
void vtol_task(void *argument);

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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_0) {

		temp_mode_flag++;
		if (temp_mode_flag == 3) {
			temp_mode_flag = 0;
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {

		itr_running = 1;

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
				/*
				 case 7:
				 ppm_live_channels[6] = pulse_width;
				 pulse++;
				 break;
				 case 8:
				 ppm_live_channels[7] = pulse_width;
				 pulse++;
				 break;
				 case 9:
				 ppm_live_channels[8] = pulse_width;
				 pulse++;
				 break;
				 case 10:
				 ppm_live_channels[9] = pulse_width;
				 pulse++;
				 break;*/

			default:
				pulse++;
				break;
			}
		}
		last_capture = current_capture;

		itr_running = 0;
	}

}

void pid_equation(float error, float p, float i, float d, float prev_error,
		float prev_iterm) {
	float pterm = p * error;
	float iterm = prev_iterm + i * (error + prev_error) * 0.0025 / 2;
	static int saturation_limit = 200;

	if (iterm > saturation_limit) {
		iterm = saturation_limit;
	} else if (iterm < -saturation_limit) {
		iterm = -saturation_limit;
	}

	float Dterm = d * (error - prev_error) / 0.0025f;
	float PID_output = pterm + iterm + Dterm;

	if (PID_output > saturation_limit) {
		PID_output = saturation_limit;
	} else if (PID_output < -saturation_limit) {
		PID_output = -saturation_limit;
	}

	PIDReturn[0] = PID_output;
	PIDReturn[1] = error;
	PIDReturn[2] = iterm;
}

void reset_pid(void) {
	prev_error_rate_roll = 0;
	prev_error_rate_pitch = 0;
	prev_error_rate_yaw = 0;

	prev_iterm_rate_roll = 0;
	prev_iterm_rate_pitch = 0;
	prev_iterm_rate_yaw = 0;

	prev_error_angle_roll = 0;
	prev_error_angle_pitch = 0;
	prev_iterm_angle_roll = 0;
	prev_iterm_angle_pitch = 0;
}

/* Pushkar, 22-05-2026
 *  we can add modes later as per modes of FCU i.e VTOL, QUAD or FXDW*/

/**
 * @brief  Function to lock the Actuators Manually
 * @note  	Use In the respective modes i.e VTOL,QUAD,FXDW , Check before every Control loop !
 * 		After triggering, a system reset is required.
 * @param  None
 * @retval None

 * NOTE : Function uses Hard infinite loop , do not use while(1)
 */

void actuator_emergency_stop_latch(void) {
	if (display_channels[9] > 1500) {

		if (mode_flag == 0) 	// Quad
				{
			M1 = 1000;
			M2 = 1000;
			M3 = 1000;
			M4 = 1000;

			set_raw_ccr(M1, 5);
			set_raw_ccr(M4, 7);
			set_raw_ccr(M3, 4);
			set_raw_ccr(M2, 6);

			arm_flag = 0;
			disarm_flag = 1;

			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);

			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);

			for (;;) {
			}

		} else if (mode_flag == 1)     // Fixed wing
				{
			arm_flag = 0;
			disarm_flag = 1;

			set_raw_ccr(1000, 6);   // Motor off
			set_raw_ccr(1500, 4);   // Elevon neutral
			set_raw_ccr(1500, 5);   // Elevon neutral

			for (;;) {
			}
		} else if (mode_flag == 2)     // VTOL
				{
			arm_flag = 0;
			disarm_flag = 1;

			for (;;) {
			}
		}
	} else {
		__NOP();
	}
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

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM9_Init();
	MX_TIM10_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART6_UART_Init();
	/* USER CODE BEGIN 2 */
	__HAL_RCC_SYSCFG_CLK_ENABLE();

	ESC_Calibrate();

	// initilization of sensors and calibration
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	mpu_init();
	HAL_Delay(100);
	bmp280_i2c_setup();

	HAL_Delay(1000);

	/*	-replaced mpu_roll_pitch_calibration_accel() , mpu_gyro_calibration() to struct-based outputs
	 -now it returns a struct with all calibration values , instead of calling multiple functions */
	accel_roll_pitch_calib_constant roll_pitch_calib_offset;
	mpu_roll_pitch_calibration_accel(&roll_pitch_calib_offset);

	calibration_const_global_roll_accel = roll_pitch_calib_offset.rc;
	calibration_const_global_pitch_accel = roll_pitch_calib_offset.pc;

	gyro_calib gyro_calib_offset;
	mpu_gyro_calibration(&gyro_calib_offset);

	calibration_const_global_gx = gyro_calib_offset.gx_offset;
	calibration_const_global_gy = gyro_calib_offset.gy_offset;
	calibration_const_global_gz = gyro_calib_offset.gz_offset;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // onboard led on

	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* creation of timer_sem */
	timer_semHandle = osSemaphoreNew(1, 1, &timer_sem_attributes);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	HAL_TIM_Base_Start_IT(&htim10);
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of ArmDisarm */
	ArmDisarmHandle = osThreadNew(arm_disarm, NULL, &ArmDisarm_attributes);

	/* creation of ModeHandler */
	ModeHandlerHandle = osThreadNew(mode_handler, NULL,
			&ModeHandler_attributes);

	/* creation of Debounce_Handle */
	Debounce_HandleHandle = osThreadNew(debounce_task, NULL,
			&Debounce_Handle_attributes);

	/* creation of QuadTask */
	// QuadTaskHandle = osThreadNew(quad_mode, NULL, &QuadTask_attributes);
	/* creation of TelemetryTask */
	TelemetryTaskHandle = osThreadNew(telemetry_task, NULL,
			&TelemetryTask_attributes);

	/* creation of FixedWingTask */
	// FixedWingTaskHandle = osThreadNew(fw_mode, NULL, &FixedWingTask_attributes);
	/* creation of VtolMode */
	//VtolModeHandle = osThreadNew(vtol_task, NULL, &VtolMode_attributes);
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
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
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
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
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 84 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 20000 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
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
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 84 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 20000 - 1;
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
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
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
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 84 - 1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM9 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM9_Init(void) {

	/* USER CODE BEGIN TIM9_Init 0 */

	/* USER CODE END TIM9_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };

	/* USER CODE BEGIN TIM9_Init 1 */

	/* USER CODE END TIM9_Init 1 */
	htim9.Instance = TIM9;
	htim9.Init.Prescaler = 84 - 1;
	htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim9.Init.Period = 65535;
	htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim9) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM9_Init 2 */

	/* USER CODE END TIM9_Init 2 */

}

/**
 * @brief TIM10 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM10_Init(void) {

	/* USER CODE BEGIN TIM10_Init 0 */

	/* USER CODE END TIM10_Init 0 */

	/* USER CODE BEGIN TIM10_Init 1 */

	/* USER CODE END TIM10_Init 1 */
	htim10.Instance = TIM10;
	htim10.Init.Prescaler = 83;
	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim10.Init.Period = 2499;
	htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim10) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM10_Init 2 */

	/* USER CODE END TIM10_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
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
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_USART6_UART_Init(void) {

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 9600;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_14, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB12 PB14 */
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_arm_disarm */
/**
 * @brief  Function implementing the ArmDisarm thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_arm_disarm */
void arm_disarm(void *argument) {
	/* USER CODE BEGIN 5 */
	int i_oce = 0;
	int j_oce = 0;

	/* Infinite loop */
	for (;;) {

		arm_disarm_task ^= 1;

		if (display_channels[4] > 1700 && i_oce != 1) {

			i_oce = 1;
			arm_flag = 1;
			disarm_flag = 0;


			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

			j_oce = 0;

		} else if (display_channels[4] < 1300 && j_oce != 1) {

			j_oce = 1;
			arm_flag = 0;
			disarm_flag = 1;
			reset_pid();

			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);

			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);

			i_oce = 0;

		} else {
			__NOP();
		}

		osDelay(1);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_mode_handler */
/**
 * @brief Function implementing the ModeHandler thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_mode_handler */
void mode_handler(void *argument) {
	/* USER CODE BEGIN mode_handler */

	int i_oce = 0;
	int j_oce = 0;
	int c_oce = 0;

	/* Infinite loop */

	for (;;) {

		uint32_t half_ms;

		if (mode_flag == 0) {
			if (!i_oce) {
				QuadTaskHandle = osThreadNew(quad_mode, NULL,
						&QuadTask_attributes);
				osThreadTerminate(FixedWingTaskHandle);
				osThreadTerminate(VtolModeHandle);
				i_oce = 1;
			}
			j_oce = 0;
			c_oce = 0;
			half_ms = 1000;
		} else if (mode_flag == 1) {
			if (!j_oce) {
				FixedWingTaskHandle = osThreadNew(fw_mode, NULL,
						&FixedWingTask_attributes);
				osThreadTerminate(QuadTaskHandle);
				osThreadTerminate(VtolModeHandle);
				j_oce = 1;
			}

			i_oce = 0;
			c_oce = 0;
			half_ms = 334;

		} else if (mode_flag == 2) {
			if (!c_oce) {
				VtolModeHandle = osThreadNew(vtol_task, NULL,
						&VtolMode_attributes);
				osThreadTerminate(QuadTaskHandle);
				osThreadTerminate(FixedWingTaskHandle);
				c_oce = 1;
			}

			i_oce = 0;
			j_oce = 0;
			half_ms = 100;

		} else {
			__NOP();
		}

		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		osDelay(half_ms);

		mode_task ^= 1;

	}
	/* USER CODE END mode_handler */
}

/* USER CODE BEGIN Header_debounce_task */
/**
 * @brief Function implementing the Debounce_Handle thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_debounce_task */
void debounce_task(void *argument) {
	/* USER CODE BEGIN debounce_task */
	/* Infinite loop */
	for (;;) {

		mode_flag = temp_mode_flag;
		osDelay(1);
		bounce_task ^= 1;

	}
	/* USER CODE END debounce_task */
}

/* USER CODE BEGIN Header_quad_mode */
/**
 * @brief Function implementing the QuadTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_quad_mode */
void quad_mode(void *argument) {
	/* USER CODE BEGIN quad_mode */
	int ThrottleIdle = 1050;
	int test_var = 0;
	HAL_TIM_Base_Start(&htim9);
	HAL_TIM_Base_Start_IT(&htim10);

	/* Infinite loop */

	for (;;) {

		quad_task ^= 1;
		static float angle_saturation_limit = 20.0f;

		osSemaphoreAcquire(timer_semHandle, osWaitForever);

		now = TIM9->CNT;

		if (now >= last) {
			dt_n = now - last;
		} else {
			dt_n = (65535 - last) + now;
		}

		last = now;

		memcpy(display_channels, (void*) ppm_ready_channels,
				sizeof(ppm_ready_channels));


		 ch1_dbg = display_channels[0];
		 ch2_dbg = display_channels[1];
		 ch3_dbg = display_channels[2];
		 ch4_dbg = display_channels[3];
		 ch5_dbg = display_channels[4];
		 ch6_dbg = display_channels[5];

		/*ch1_dbg = range_converter(display_channels[0], 973, 1943, 1000, 2000);
		 ch2_dbg = range_converter(display_channels[1], 973, 1943, 1000, 2000);*/

		if (arm_flag == 1 && disarm_flag == 0) { //stabilize mode

			if (!test_var) {

				in_stabilize = 1;
				in_rate = 0;

				deadband_filter_temp_ch1 = range_converter(display_channels[0],
						973, 1943, 1000, 2000);
				deadband_filter_temp_ch2 = range_converter(display_channels[1],
						973, 1943, 1000, 2000);


				if ((deadband_filter_temp_ch1 <= 1510)
						&& (deadband_filter_temp_ch1 >= 1490)) {
					deadband_filter_temp_ch1 = 1500;
				}

				if ((deadband_filter_temp_ch2 <= 1510)
						&& (deadband_filter_temp_ch2 >= 1490)) {
					deadband_filter_temp_ch2 = 1500;
				}

				desired_angle_roll = 0.25 * (deadband_filter_temp_ch1 - 1500);
				desired_angle_pitch = 0.25 * (deadband_filter_temp_ch2 - 1500);

				if (desired_angle_roll > angle_saturation_limit) {
					desired_angle_roll = angle_saturation_limit;
				}
				if (desired_angle_roll < -angle_saturation_limit) {
					desired_angle_roll = -angle_saturation_limit;
				}

				if (desired_angle_pitch > angle_saturation_limit) {
					desired_angle_pitch = angle_saturation_limit;
				}
				if (desired_angle_pitch < -angle_saturation_limit) {
					desired_angle_pitch = -angle_saturation_limit;
				}

				deadband_filter_temp_ch3 = range_converter(display_channels[2],
									973, 1943, 1000, 2000);
				input_throttle = deadband_filter_temp_ch3;
				desired_rate_yaw = 0.25 * (display_channels[3] - 1500);

				desired_rate_yaw = -desired_rate_yaw;

				//stabilize controller
				mpu_get_kalman_angles(&kalman_roll, &kalman_pitch);

				error_angle_roll = desired_angle_roll - kalman_roll;
				error_angle_pitch = desired_angle_pitch - (-kalman_pitch);

				p_angle_roll = map_rc_to_pid(display_channels[5]);
				p_angle_pitch = map_rc_to_pid(display_channels[5]);

				pid_equation(error_angle_roll, p_angle_roll, i_angle_roll,
						d_angle_roll, prev_error_angle_roll,
						prev_iterm_angle_roll);

				desired_rate_roll = PIDReturn[0];
				prev_error_angle_roll = PIDReturn[1];
				prev_iterm_angle_roll = PIDReturn[2];

				pid_equation(error_angle_pitch, p_angle_pitch, i_angle_pitch,
						d_angle_pitch, prev_error_angle_pitch,
						prev_iterm_angle_pitch);

				desired_rate_pitch = PIDReturn[0];
				prev_error_angle_pitch = PIDReturn[1];
				prev_iterm_angle_pitch = PIDReturn[2];

				mpu_gyro_raw raw_gyro;
				mpu_gyro_read(&raw_gyro);

				//rate controller
				error_rate_roll = desired_rate_roll
						- (raw_gyro.Gx - calibration_const_global_gx); //x

				error_rate_pitch = desired_rate_pitch
						- (raw_gyro.Gy - calibration_const_global_gy); //y

				error_rate_yaw = desired_rate_yaw
						- (raw_gyro.Gz - calibration_const_global_gz); //z

				pid_equation(error_rate_roll, p_rate_roll, i_rate_roll,
						d_rate_roll, prev_error_rate_roll,
						prev_iterm_rate_roll);

				input_roll = PIDReturn[0];
				prev_error_rate_roll = PIDReturn[1];
				prev_iterm_rate_roll = PIDReturn[2];

				pid_equation(error_rate_pitch, p_rate_pitch, i_rate_pitch,
						d_rate_pitch, prev_error_rate_pitch,
						prev_iterm_rate_pitch);

				input_pitch = PIDReturn[0];
				prev_error_rate_pitch = PIDReturn[1];
				prev_iterm_rate_pitch = PIDReturn[2];

				pid_equation(error_rate_yaw, p_rate_yaw, i_rate_yaw, d_rate_yaw,
						prev_error_rate_yaw, prev_iterm_rate_yaw);

				input_yaw = PIDReturn[0];
				prev_error_rate_yaw = PIDReturn[1];
				prev_iterm_rate_yaw = PIDReturn[2];

				M1 =
						1.024f
								* (input_throttle - input_roll - input_pitch
										- input_yaw);
				M2 =
						1.024f
								* (input_throttle - input_roll + input_pitch
										+ input_yaw);
				M3 =
						1.024f
								* (input_throttle + input_roll + input_pitch
										- input_yaw);
				M4 =
						1.024f
								* (input_throttle + input_roll - input_pitch
										+ input_yaw);

				if (M1 < ThrottleIdle) {
					M1 = ThrottleIdle;
				}
				if (M2 < ThrottleIdle) {
					M2 = ThrottleIdle;
				}
				if (M3 < ThrottleIdle) {
					M3 = ThrottleIdle;
				}
				if (M4 < ThrottleIdle) {
					M4 = ThrottleIdle;
				}
				if (M1 > rc_max_us) {
					M1 = rc_max_us - 1;
				}
				if (M2 > rc_max_us) {
					M2 = rc_max_us - 1;
				}
				if (M3 > rc_max_us) {
					M3 = rc_max_us - 1;
				}
				if (M4 > rc_max_us) {
					M4 = rc_max_us - 1;
				}

				int ThrottleCutOff = rc_min_us;

				if (display_channels[2] < 1050) {
					M1 = ThrottleCutOff;
					M2 = ThrottleCutOff;
					M3 = ThrottleCutOff;
					M4 = ThrottleCutOff;
					reset_pid();
				}

				set_raw_ccr(M1, 5);
				set_raw_ccr(M4, 3);
				set_raw_ccr(M3, 1);
				set_raw_ccr(M2, 6);

			} else if (test_var) {

				in_stabilize = 0;
				in_rate = 1;

				deadband_filter_temp_ch1 = range_converter(display_channels[0],
						973, 1943, 1000, 2000);
				deadband_filter_temp_ch2 = range_converter(display_channels[1],
						973, 1943, 1000, 2000);

				if ((deadband_filter_temp_ch1 <= 1510)
						&& (deadband_filter_temp_ch1 >= 1490)) {
					deadband_filter_temp_ch1 = 1500;
				}

				if ((deadband_filter_temp_ch2 <= 1510)
						&& (deadband_filter_temp_ch2 >= 1490)) {
					deadband_filter_temp_ch2 = 1500;
				}

				desired_rate_roll = 0.25 * (deadband_filter_temp_ch1 - 1500); // Earlier was 0.15
				desired_rate_pitch = 0.25 * (deadband_filter_temp_ch2 - 1500);
				input_throttle = display_channels[2];
				desired_rate_yaw = 0.25 * (display_channels[3] - 1500);

				desired_rate_yaw = -desired_rate_yaw;

				mpu_gyro_raw raw_gyro;
				mpu_gyro_read(&raw_gyro);

				error_rate_roll = desired_rate_roll
						- (raw_gyro.Gx - calibration_const_global_gx); //x
				error_rate_pitch = desired_rate_pitch
						- (raw_gyro.Gy - calibration_const_global_gy); //y
				error_rate_yaw = desired_rate_yaw
						- (raw_gyro.Gz - calibration_const_global_gz); //z

				/* p_rate_roll = map_rc_to_pid(display_channels[5]);
				 p_rate_pitch = map_rc_to_pid(display_channels[5]);*/

				pid_equation(error_rate_roll, p_rate_roll, i_rate_roll,
						d_rate_roll, prev_error_rate_roll,
						prev_iterm_rate_roll);
				input_roll = PIDReturn[0];
				prev_error_rate_roll = PIDReturn[1];
				prev_iterm_rate_roll = PIDReturn[2];

				pid_equation(error_rate_pitch, p_rate_pitch, i_rate_pitch,
						d_rate_pitch, prev_error_rate_pitch,
						prev_iterm_rate_pitch);
				input_pitch = PIDReturn[0];
				prev_error_rate_pitch = PIDReturn[1];
				prev_iterm_rate_pitch = PIDReturn[2];

				pid_equation(error_rate_yaw, p_rate_yaw, i_rate_yaw, d_rate_yaw,
						prev_error_rate_yaw, prev_iterm_rate_yaw);
				input_yaw = PIDReturn[0];
				prev_error_rate_yaw = PIDReturn[1];
				prev_iterm_rate_yaw = PIDReturn[2];

				M1 =
						1.024f
								* (input_throttle - input_roll - input_pitch
										- input_yaw);
				M2 =
						1.024f
								* (input_throttle - input_roll + input_pitch
										+ input_yaw);
				M3 =
						1.024f
								* (input_throttle + input_roll + input_pitch
										- input_yaw);
				M4 =
						1.024f
								* (input_throttle + input_roll - input_pitch
										+ input_yaw);

				if (M1 < ThrottleIdle) {
					M1 = ThrottleIdle;
				}
				if (M2 < ThrottleIdle) {
					M2 = ThrottleIdle;
				}
				if (M3 < ThrottleIdle) {
					M3 = ThrottleIdle;
				}
				if (M4 < ThrottleIdle) {
					M4 = ThrottleIdle;
				}

				// capping maximum m1-4 values - aryan

				if (M1 > rc_max_us) {
					M1 = rc_max_us;
				}
				if (M2 > rc_max_us) {
					M2 = rc_max_us;
				}
				if (M3 > rc_max_us) {
					M3 = rc_max_us;
				}
				if (M4 > rc_max_us) {
					M4 = rc_max_us;
				}

				int ThrottleCutOff = 1000;

				if (display_channels[2] < 1050) {
					M1 = ThrottleCutOff;
					M2 = ThrottleCutOff;
					M3 = ThrottleCutOff;
					M4 = ThrottleCutOff;
					reset_pid();
				}

				set_raw_ccr(M1, 5);
				set_raw_ccr(M4, 3);
				set_raw_ccr(M3, 1);
				set_raw_ccr(M2, 6);

			}

		}

		//for debugging on MCUViewer and For giving time exceeded warnings in mavlink

	}

	/* USER CODE END quad_mode */
}

/* USER CODE BEGIN Header_telemetry_task */
/**
 * @brief Function implementing the TelemetryTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_telemetry_task */
void telemetry_task(void *argument) {
	/* USER CODE BEGIN telemetry_task */

	/* Infinite loop */
	for (;;) {

		telemetery_task ^= 1;

		if (arm_flag == 1 && disarm_flag == 0) {

			//alt_ext = bmp280_get_altitude();
			printf("p pitch: %f\n", p_angle_pitch);
			//printf("I roll: %f\n", i_rate_roll);
			/*
			 send_heartbeat_armed();
			 send_attitude();
			 send_battery_info();
			 send_status_text(MAV_SEVERITY_INFO, "ARMED!");*/

		}

		else if (arm_flag == 0 && disarm_flag == 1) {
			/*	-replaced mpu_accel_read() , mpu_gyro_read() to struct-based outputs
			 -now it returns a struct with all calibration values , instead of calling multiple functions */

			printf("p pitch: %f\n", p_angle_pitch);
			//printf("i roll: %f\n", p_rate_roll);

			mpu_accel_raw raw_accel;
			mpu_accel_read(&raw_accel);
			mpu_gyro_raw raw_gyro;
			mpu_gyro_read(&raw_gyro);

			mpu_get_kalman_angles(&kalman_roll, &kalman_pitch);
			g_global_x = raw_gyro.Gx;
			g_global_y = raw_gyro.Gy;
			g_global_z = raw_gyro.Gz;
			a_global_x = raw_accel.Ax;
			a_global_y = raw_accel.Ay;
			a_global_z = raw_accel.Az;
			alt_ext = bmp280_get_altitude();
			/*send_heartbeat_disarmed();
			 send_attitude();
			 send_scaled_pressure();*/

			//printf("Telemetry is alive!\n");
			//send_battery_info();
			send_status_text(MAV_SEVERITY_INFO, "DISARMED!");

		}

		osDelay(500); // NOTE : Changed to 500 ms so doesn't consume a lot of cycles

	}
	/* USER CODE END telemetry_task */
}

/* USER CODE BEGIN Header_fw_mode */
/**
 * @brief Function implementing the FixedWingTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_fw_mode */
void fw_mode(void *argument) {
	/* USER CODE BEGIN fw_mode */
	/* Infinite loop */
	float Gx, Gy;
	float left_elevon;
	float right_elevon;

	float throttle;
	for (;;) {
		actuator_emergency_stop_latch();

		osSemaphoreAcquire(timer_semHandle, osWaitForever);

		if (display_channels[6] > 1900) {

			mpu_get_kalman_angles(&kalman_roll, &kalman_pitch);

			desired_angle_roll = 0.05f * (display_channels[0] - 1500);
			desired_angle_pitch = 0.07f * (display_channels[1] - 1500);

			error_angle_roll = desired_angle_roll - kalman_roll;
			error_angle_pitch = desired_angle_pitch - kalman_pitch;

			desired_rate_roll = p_angle_roll * error_angle_roll;
			desired_rate_pitch = p_angle_pitch * error_angle_pitch;

			static float rate_saturation_limit = 200.0f;

			if (desired_rate_roll > rate_saturation_limit) {
				desired_rate_roll = rate_saturation_limit - 0.1f;
			}
			if (desired_rate_roll < -rate_saturation_limit) {
				desired_rate_roll = -rate_saturation_limit + 0.1f;
			}

			if (desired_rate_pitch > rate_saturation_limit) {
				desired_rate_pitch = rate_saturation_limit - 0.1f;
			}
			if (desired_rate_pitch < -rate_saturation_limit) {
				desired_rate_pitch = -rate_saturation_limit + 0.1f;
			}

			mpu_gyro_raw raw_gyro;
			mpu_gyro_read(&raw_gyro);

			Gx = raw_gyro.Gx - calibration_const_global_gx;
			Gy = raw_gyro.Gy - calibration_const_global_gy;

			error_rate_roll = desired_rate_roll - Gx;
			error_rate_pitch = desired_rate_pitch - Gy;

			pid_equation(error_rate_roll, p_rate_roll, i_rate_roll, d_rate_roll,
					prev_error_rate_roll, prev_iterm_rate_roll);

			input_roll = PIDReturn[0];
			prev_error_rate_roll = PIDReturn[1];
			prev_iterm_rate_roll = PIDReturn[2];

			pid_equation(error_rate_pitch, p_rate_pitch, i_rate_pitch,
					d_rate_pitch, prev_error_rate_pitch, prev_iterm_rate_pitch);

			input_pitch = PIDReturn[0];
			prev_error_rate_pitch = PIDReturn[1];
			prev_iterm_rate_pitch = PIDReturn[2];

			elevon_left = 1500 + (EL_L_DIR * (input_pitch + input_roll));
			elevon_right = 1500 + (EL_R_DIR * (input_pitch - input_roll));

			// Clamp
			if (elevon_left > rc_max_us) {
				elevon_left = rc_max_us - 1;
			}
			if (elevon_left < rc_min_us) {
				elevon_left = rc_min_us + 1;
			}

			if (elevon_right > rc_max_us) {
				elevon_right = rc_max_us - 1;
			}
			if (elevon_right < rc_min_us) {
				elevon_right = rc_min_us + 1;
			}

			set_raw_ccr(elevon_left, 4);
			set_raw_ccr(elevon_right, 5);

			input_throttle = display_channels[2];
			set_raw_ccr(input_throttle, 6);

		} else if (display_channels[7] < 1300) {

			left_elevon = range_converter(display_channels[0], rc_min_us,
					rc_max_us, 5, 10);
			right_elevon = range_converter(display_channels[1], rc_min_us,
					rc_max_us, 5, 10);
			throttle = range_converter(display_channels[2], rc_min_us,
					rc_max_us, 5, 10);

			set_raw_ccr(throttle, 6);
			set_raw_ccr(left_elevon, 4);
			set_raw_ccr(right_elevon, 5);

		}

	}
	/* USER CODE END fw_mode */
}

/* USER CODE BEGIN Header_vtol_task */
/**
 * @brief Function implementing the VtolMode thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_vtol_task */
void vtol_task(void *argument) {
	/* USER CODE BEGIN vtol_task */
	/* Infinite loop */
	for (;;) {
		actuator_emergency_stop_latch();
		osDelay(1);
	}
	/* USER CODE END vtol_task */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM11 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM11) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */
	if (htim->Instance == TIM10) {
		tim_flag = 1;
		osSemaphoreRelease(timer_semHandle);
	}
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
