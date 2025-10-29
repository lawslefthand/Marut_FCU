/*
 * qmc5883l.c
 *
 *  Created on: Oct 22, 2025
 *      Author: danba
 */

#include <math.h>
#include <stdio.h>
#include <stm32f4xx.h>
#include <qmcr5883l.h>


/*
 * =================================================================================
 * == Magnetometer Register Map Differences (QMC5883P vs. HMC5883L vs. QMC5883L) ==
 * =================================================================================
 *
 * WARNING: These chips are NOT drop-in replacements for each other.
 * Code written for one will NOT work on the others without significant changes.
 * The HMC5883L is the original Honeywell part. The QMC5883L and QMC5883P
 * are common but incompatible clones.
 *
 * ---------------------------------------------------------------------------------
 * Feature          | QMC5883P (This PDF)   | HMC5883L (Honeywell)  | QMC5883L (Clone)
 * ---------------------------------------------------------------------------------
 * I2C Slave Addr   | 0x2C                  | 0x1E                  | 0x2C
 *
 * Chip ID Register | 0x00                  | 0x0A (Ident Reg A)    | 0x0D
 *
 * Chip ID Value    | 0x80                  | 0x48 (ASCII 'H')      | 0xFF
 *
 * Data Registers   | 0x01 - 0x06           | 0x03 - 0x08           | 0x00 - 0x05
 *
 * Data Order       | X (LSB, MSB)          | X (MSB, LSB)          | X (LSB, MSB)
 * | Y (LSB, MSB)          | Z (MSB, LSB) !!       | Y (LSB, MSB)
 * | Z (LSB, MSB)          | Y (MSB, LSB) !!       | Z (LSB, MSB)
 * |                       |                       |
 * | // Note: HMC5883L swaps Z and Y axes in its registers!
 *
 * Control Reg 1    | 0x0A                  | 0x00 (Config A)       | 0x09
 *
 * Control Reg 2    | 0x0B                  | 0x01 (Config B)       | 0x0A
 *
 * Mode Register    | (Bits in 0x0A)        | 0x02 (Mode)           | (Bits in 0x09)
 *
 * Status Register  | 0x09                  | 0x09                  | 0x06
 * ---------------------------------------------------------------------------------
 */

extern I2C_HandleTypeDef hi2c1;


void qmc_i2c_write(uint8_t reg_addr, uint8_t value) {
    HAL_I2C_Mem_Write(&hi2c1, (QMC_ADDR << 1), reg_addr, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}

uint8_t qmc_i2c_read(uint8_t reg_addr) {
    uint8_t value = 0;
    HAL_I2C_Mem_Read(&hi2c1, (QMC_ADDR << 1), reg_addr, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
    return value;
}



float qmc_mag_read(void) {

	float mag_x, mag_y, mag_z = 0;
	uint8_t Rec_Data[6];
	float angle = 0;

	Rec_Data[0] = qmc_i2c_read(0x02); // Read XOUT MSB
	Rec_Data[1] = qmc_i2c_read(0x01); // Read XOUT LSB
	Rec_Data[2] = qmc_i2c_read(0x04); // Read YOUT MSB
	Rec_Data[3] = qmc_i2c_read(0x03); // Read YOUT LSB
	Rec_Data[4] = qmc_i2c_read(0x06); // Read ZOUT MSB
	Rec_Data[5] = qmc_i2c_read(0x05); // Read ZOUT LSB

	mag_x = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	mag_y = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	mag_z = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);



	//printf("mag: X=%.2f , Y=%.2f , Z=%.2f \r\n", mag_x, mag_y, mag_z);

	angle = atan2(mag_y, mag_x);

	 angle = angle * (180.0f / 3.14159f);

	if (angle <0)
	{
		angle+=360.0f;
	}

	//printf("Heading is %f\n",angle);

	return angle;

}




void qmc_init(void) {
	uint8_t check;

	// Read Chip ID
	check = qmc_i2c_read(0x00);
	printf("ID is 0x%X\r\n", check);

	// Chip id may vary with clones. Please check datasheet and verify on own caution.
	if (check == 0x80) {
		printf("Correct sensor ID\r\n");

		/*
		 * Based on Datasheet "Normal Mode Setup Example"
		 */

		// 0x08 to Register 0BH
		// Field Range to 8 Gauss [cite: 403, 585]
		qmc_i2c_write(0x0B, 0x08);

		// 0xCD to Register 0AH
		// Normal Mode, ODR=200Hz, OSR1=8, OSR2=8
		qmc_i2c_write(0x0A, 0xCD);

	} else {
		printf("Invalid sensor ID\r\n");
	}
}




