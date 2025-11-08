/*
 * bmp280.h
 *
 *  Created on: Oct 28, 2025
 *      Author: danba
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#define BMP280_I2C_ADDR (0x76 << 1)



#include "stm32f4xx.h"
#include <math.h>

void bmp_hal_i2c_write(uint8_t reg_addr, uint8_t value);
uint8_t bmp_hal_i2c_read(uint8_t reg_addr);
double temperature(int x);
double pressure(void);
void bmp_i2c_setup(void);
float altitude_calc(void);

#endif /* INC_BMP280_H_ */
