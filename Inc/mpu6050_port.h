/*
 * mpu6050_port.h
 *
 *  Created on: Oct 9, 2025
 *      Author: danba
 */

#ifndef INC_MPU6050_PORT_H_
#define INC_MPU6050_PORT_H_

/*
 * mpu6050_hal.h
 *
 * Created on: Oct 9, 2025
 * Author: YourName
 */

#ifndef MPU6050_HAL_H_
#define MPU6050_HAL_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define MPU6050_ADDR        (0x68 << 1)
#define MPU9250_MAG_ADDRESS 0x0C

#define WHO_AM_I_REG        0x75
#define PWR_MGMT_1_REG      0x6B
#define SMPLRT_DIV_REG      0x19
#define GYRO_CONFIG_REG     0x1B
#define ACCEL_CONFIG_REG    0x1C
#define ACCEL_XOUT_H_REG    0x3B
#define GYRO_XOUT_H_REG     0x43

#define CNTLA 0x0A
#define INT_PIN_CFG 0x37


extern I2C_HandleTypeDef hi2c2;
extern int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
extern int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;

extern float Ax, Ay, Az;

extern float Gx, Gy, Gz;

extern float mag_x, mag_y, mag_z;

extern float mag_adj_x, mag_adj_y, mag_adj_z;

void mpu_init(void);

void mpu_accel_read(void);

void mpu_gyro_read(void);

float mpu_accel_read_x(void);

float mpu_accel_read_y(void);

float mpu_accel_read_z(void);

float mpu_gyro_read_x(void);

float mpu_gyro_read_y(void);

float mpu_gyro_read_z(void);

#endif /* MPU6050_HAL_H_ */

#endif /* INC_MPU6050_PORT_H_ */
