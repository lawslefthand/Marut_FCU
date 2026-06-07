/*
 * Support_function.h
 *
 *  Created on: Jun 1, 2026
 *      Author: Siddhu
 */

#ifndef INC_SUPPORT_FUNCTION_H_
#define INC_SUPPORT_FUNCTION_H_

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

uint32_t map_rc_to_motor(uint16_t rcValue);
float negative_range(uint32_t motor_value);
float negative_range(uint32_t motor_value);
void set_servo_angle(uint32_t angle, int channel);
float range_converter(uint16_t old_value, uint16_t old_min, uint16_t old_max,
		float new_min, float new_max);
void set_raw_ccr(int ccr_val, int channel);
uint16_t map_rc_to_pid(uint16_t rc_value, uint16_t ch);
void motor_check(void);
void ESC_Calibrate(void);

#endif /* INC_SUPPORT_FUNCTION_H_ */
