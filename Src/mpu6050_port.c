#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "mpu6050_port.h"


void mpu_init(void) {
	uint8_t check;
	uint8_t data;

	// Check device ID
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1,
	HAL_MAX_DELAY);
	printf("ID is %u\r\n", check);

	if (check == 0x68) {
		printf("Correct sensor ID\r\n");

		data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1,
		HAL_MAX_DELAY);

		// data rate = 1KHz
		data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1,
		HAL_MAX_DELAY);

		// Full Scale Range +/- 250dps (gyro)
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1,
		HAL_MAX_DELAY);

		// Full Scale Range +/- 2g (accel)
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1,
		HAL_MAX_DELAY);

	} else {
		printf("Invalid sensor ID\r\n");
	}
}

float mpu_accel_read(int ret) {
	uint8_t Rec_Data[6];

	int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;

	float Ax, Ay, Az;


	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6,
	HAL_MAX_DELAY);

	Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

	// For AFS_SEL=0, sensitivity is 16384 LSB/g
	Ax = (float) Accel_X_RAW / 16384.0f;
	Ay = (float) Accel_Y_RAW / 16384.0f;
	Az = (float) Accel_Z_RAW / 16384.0f;

	//this formula basically reads and 'combines' the individual acceleration components of the x,y,z components
	//think of it as each axis having its downwards gravitational component and then accounting for all the respective components
	//to get your roll and pitch relative to downwards gravity vector - aryan
	float Roll = atan2f(Ay, sqrtf(Ax * Ax + Az * Az)) * 180.0f / 3.14159f;
	float Pitch = atan2f(-Ax, sqrtf(Ay * Ay + Az * Az)) * 180.0f / 3.14159f;

	if (ret==0)
	{
		return Roll;
	}
	else if (ret==1)
	{
		return Pitch;
	}
	else
	{
		return 0.0;
	}


	//printf("Roll is %f and pitch is %f\n", Roll, Pitch);
}

void mpu_gyro_read(void) {
	uint8_t Rec_Data[6];

	int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6,
	HAL_MAX_DELAY);

	Gyro_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	Gyro_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	Gyro_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

	// For FS_SEL=0, sensitivity is 131 LSB/dps
	Gx = (float) Gyro_X_RAW / 131.0f;
	Gy = (float) Gyro_Y_RAW / 131.0f;
	Gz = (float) Gyro_Z_RAW / 131.0f;

}



