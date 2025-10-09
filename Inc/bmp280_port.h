/*
 * bmp280_port.h
 *
 *  Created on: Oct 9, 2025
 *      Author: danba
 */

#ifndef INC_BMP280_PORT_H_
#define INC_BMP280_PORT_H_

/*
 * bmp280_hal.h
 *
 * Created on: Oct 9, 2025
 * Author: danba
 */

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define BMP280_I2C_ADDR (0x76 << 1)

extern I2C_HandleTypeDef hi2c1;

void bmp_i2c_setup(void);

double temperature(int x);

double pressure(void);

double altitude(double press);

void bmp_hal_i2c_write(uint8_t reg_addr, uint8_t value);

uint8_t bmp_hal_i2c_read(uint8_t reg_addr);

#endif /* INC_BMP280_PORT_H_ */
