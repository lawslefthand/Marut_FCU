/*
 * bmp280.c
 *
 *  Created on: Oct 28, 2025
 *      Author: danba
 */

#include <bmp280.h>

extern I2C_HandleTypeDef hi2c1;

void bmp_hal_i2c_write(uint8_t reg_addr, uint8_t value) {
	uint8_t data_to_write = value;
	HAL_I2C_Mem_Write(&hi2c1, BMP280_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT,
			&data_to_write, 1, HAL_MAX_DELAY);
}

uint8_t bmp_hal_i2c_read(uint8_t reg_addr) {
	uint8_t read_value = 0;
	HAL_I2C_Mem_Read(&hi2c1, BMP280_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT,
			&read_value, 1, HAL_MAX_DELAY);
	return read_value;
}

double temperature(int x) {
	unsigned short calib_T1 = 0;
	signed short calib_T2 = 0;
	signed short calib_T3 = 0;
	signed long raw_temperature = 0;
	double var1 = 0;
	double var2 = 0;
	uint32_t t_fine = 0;
	double final_temp = 0;

	uint8_t t1_lsb = bmp_hal_i2c_read(0x88);
	uint8_t t1_msb = bmp_hal_i2c_read(0x89);
	calib_T1 = (t1_msb << 8) | t1_lsb;

	uint8_t t2_lsb = bmp_hal_i2c_read(0x8A);
	uint8_t t2_msb = bmp_hal_i2c_read(0x8B);
	calib_T2 = (t2_msb << 8) | t2_lsb;

	uint8_t t3_lsb = bmp_hal_i2c_read(0x8C);
	uint8_t t3_msb = bmp_hal_i2c_read(0x8D);
	calib_T3 = (t3_msb << 8) | t3_lsb;

	uint8_t temp_msb = bmp_hal_i2c_read(0xFA);
	uint8_t temp_lsb = bmp_hal_i2c_read(0xFB);
	uint8_t temp_xlsb = bmp_hal_i2c_read(0xFC);
	raw_temperature = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4);

	var1 = (((raw_temperature / 16384.0) - (calib_T1 / 1024.0)) * calib_T2);
	var2 = (((raw_temperature / 131072.0) - (calib_T1 / 8192.0))
			* ((raw_temperature / 131072.0) - (calib_T1 / 8192.0)) * calib_T3);

	t_fine = var1 + var2;
	final_temp = t_fine / 5120.0;

	if (x == 1) {
		return t_fine;
	} else if (x == 0) {
		return final_temp;
	} else {
		return 0;
	}
}

double pressure(void) {
	double var1 = 0;
	double var2 = 0;
	unsigned short dig_P1 = 0;
	signed short dig_P2 = 0;
	signed short dig_P3 = 0;
	signed short dig_P4 = 0;
	signed short dig_P5 = 0;
	signed short dig_P6 = 0;
	signed short dig_P7 = 0;
	signed short dig_P8 = 0;
	signed short dig_P9 = 0;
	uint32_t t_fine = 0;
	signed long raw_pressure = 0;
	double p = 0;

	t_fine = temperature(1);

	dig_P1 = (bmp_hal_i2c_read(0x8F) << 8) | bmp_hal_i2c_read(0x8E);
	dig_P2 = (bmp_hal_i2c_read(0x91) << 8) | bmp_hal_i2c_read(0x90);
	dig_P3 = (bmp_hal_i2c_read(0x93) << 8) | bmp_hal_i2c_read(0x92);
	dig_P4 = (bmp_hal_i2c_read(0x95) << 8) | bmp_hal_i2c_read(0x94);
	dig_P5 = (bmp_hal_i2c_read(0x97) << 8) | bmp_hal_i2c_read(0x96);
	dig_P6 = (bmp_hal_i2c_read(0x99) << 8) | bmp_hal_i2c_read(0x98);
	dig_P7 = (bmp_hal_i2c_read(0x9B) << 8) | bmp_hal_i2c_read(0x9A);
	dig_P8 = (bmp_hal_i2c_read(0x9D) << 8) | bmp_hal_i2c_read(0x9C);
	dig_P9 = (bmp_hal_i2c_read(0x9F) << 8) | bmp_hal_i2c_read(0x9E);

	uint8_t press_msb = bmp_hal_i2c_read(0xF7);
	uint8_t press_lsb = bmp_hal_i2c_read(0xF8);
	uint8_t press_xlsb = bmp_hal_i2c_read(0xF9);
	raw_pressure = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4);

	var1 = ((double) t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((double) dig_P6) / 32768.0;
	var2 = var2 + (var1 * ((double) dig_P5) * 2.0);
	var2 = (var2 / 4.0) + (((double) dig_P4) * 65536.0);
	var1 = (((double) dig_P3) * var1 * var1 / 524288.0
			+ ((double) dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((double) dig_P1);

	if (var1 == 0.0) {
		return 0;
	}

	p = 1048576.0 - (double) raw_pressure;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double) dig_P9) * p * p / 2147483648.0;
	var2 = p * ((double) dig_P8) / 32768.0;
	p = p + (var1 + var2 + ((double) dig_P7)) / 16.0;

	return p;
}

void bmp_i2c_setup(void) {

	bmp_hal_i2c_write(0xF4, 0x27);

	bmp_hal_i2c_write(0xF5, 0x0C);
}

float altitude_calc(void) {
	static float refPressure = 0.0f;
	float pressure_hPa = pressure();
	float tempC = temperature(0);

	if (refPressure == 0.0f) {
		refPressure = pressure_hPa;
	}

	float altitude = ((powf((refPressure / pressure_hPa), (1.0f / 5.257f))
			- 1.0f) * (tempC + 273.15f)) / 0.0065f;

	return altitude;
}
