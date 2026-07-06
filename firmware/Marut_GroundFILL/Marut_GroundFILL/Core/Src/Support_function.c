/*
 * Support_function.c
 *
 *  Created on: Jun 1, 2026
 *      Author: Siddhu
 */
#include <Support_function.h>

int rc_max_us = 2000;
int rc_min_us = 1000;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;

extern int m_t;

uint32_t map_rc_to_motor(uint16_t rcValue) {



	if (rcValue > rc_max_us) {
		rcValue = rc_max_us;
	}
	if (rcValue < rc_min_us) {
		rcValue = rc_min_us;
	}

	uint32_t motor_value = (uint32_t) (rcValue - rc_min_us) * 180 / (rc_max_us - rc_min_us);
	return motor_value;

}

float negative_range(uint32_t motor_value)
{
	return (((float) motor_value - 90.0f) / 90.0f);
}

float range_converter(uint16_t old_value, uint16_t old_min, uint16_t old_max,
                      float new_min, float new_max)
{
    if (old_max <= old_min)
        return new_min;

    if (old_value < old_min)
        old_value = old_min;
    else if (old_value > old_max)
        old_value = old_max;

    return new_min +
           ((float)((int32_t)old_value - (int32_t)old_min) *
            (new_max - new_min)) /
           ((float)old_max - (float)old_min);
}


void set_servo_angle(uint32_t angle, int channel)
{

	if (angle > 180)
	{
		angle = 180;
	}

	uint32_t min_pulse_width = 500;
	uint32_t max_pulse_width = 2500;

	uint32_t pulse = ((angle * (max_pulse_width - min_pulse_width)) / 180)
			+ min_pulse_width;

	if (channel == 1)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
	}
	else if (channel == 2)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
	}
	else if (channel == 3)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse);
	}
	else if (channel == 4)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
	}
	else if (channel == 5)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);
	}
	else if (channel == 6)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse);
	}
	else if (channel == 7)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulse);
	}
}

void set_raw_ccr(int ccr_val, int channel)
{
	if (channel == 1)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr_val);
	}
	else if (channel == 2)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr_val);
	}
	else if (channel == 3)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ccr_val);
	}
	else if (channel == 4)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr_val);
	}
	else if (channel == 5)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ccr_val);
	}
	else if (channel == 6)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ccr_val);
	}
	else if (channel == 7)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ccr_val);
	}
}

float map_rc_to_pid(uint16_t rc_value) {
	if (rc_value > rc_max_us)
	{
		rc_value = rc_max_us;
	}
	if (rc_value < rc_min_us)
	{
		rc_value = rc_min_us;
	}
	return (rc_value - rc_min_us) * 10.0f / (rc_max_us - rc_min_us);
	//return (rc_value - 1000) * 2.0f / 1000.0f;
}

void motor_check(void) {
	int pwm;
	int start_pwm = 800;
	int target_pwm = 1500;
	int step = 1;

	/* Start PWM if not already running */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	for (pwm = start_pwm; pwm <= target_pwm; pwm += step) {
		set_raw_ccr(pwm, 1);
		set_raw_ccr(pwm, 2);
		set_raw_ccr(pwm, 3);
		set_raw_ccr(pwm, 5);
		set_raw_ccr(pwm, 6);
		set_raw_ccr(pwm, 4);
		set_raw_ccr(pwm, 7);

		m_t = pwm;

		HAL_Delay(20); // slow ramp
	}

	while (1) {
		set_raw_ccr(pwm, 1);
		set_raw_ccr(pwm, 2);
		set_raw_ccr(pwm, 3);
		set_raw_ccr(target_pwm, 5);
		set_raw_ccr(target_pwm, 6);
		set_raw_ccr(target_pwm, 4);
		set_raw_ccr(target_pwm, 7);

		HAL_Delay(20);

	}
}

void ESC_Calibrate(void)
{
	int low = 1000 ;
	int high = 2000 ;
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	set_raw_ccr(high, 5);
	set_raw_ccr(high, 6);
	set_raw_ccr(high, 4);
	set_raw_ccr(high, 7);

	HAL_Delay(3000);

	set_raw_ccr(low, 5);
	set_raw_ccr(low, 6);
	set_raw_ccr(low, 4);
	set_raw_ccr(low, 7);

	HAL_Delay(3000);


	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);



}

