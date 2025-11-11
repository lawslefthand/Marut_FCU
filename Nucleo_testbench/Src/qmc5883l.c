/*
 * qmc5883l.c
 *
 *  Created on: Oct 22, 2025
 *      Author: danba
 */

#include <math.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "qmcr5883l.h"
#include "mpu6050_port.h"

extern I2C_HandleTypeDef hi2c1;

int offset_calibrated = 0;      // 0 = not yet zeroed, 1 = done
float heading_offset = 0.0f;    // startup reference heading

// ------------------------- I2C helpers -----------------------------

void qmc_i2c_write(uint8_t reg_addr, uint8_t value) {
    HAL_I2C_Mem_Write(&hi2c1, (QMC_ADDR << 1), reg_addr,
                      I2C_MEMADD_SIZE_8BIT, &value, 1, 50);
}

uint8_t qmc_i2c_read(uint8_t reg_addr) {
    uint8_t value = 0;
    HAL_I2C_Mem_Read(&hi2c1, (QMC_ADDR << 1), reg_addr,
                     I2C_MEMADD_SIZE_8BIT, &value, 1, 50);
    return value;
}

// ---------------------- Heading correction -------------------------

float calibrate_heading(float current_heading, float heading_offset) {
    float calibrated_heading = current_heading - heading_offset;

    if (calibrated_heading < 0.0f)
        calibrated_heading += 360.0f;
    else if (calibrated_heading >= 360.0f)
        calibrated_heading -= 360.0f;

    return calibrated_heading;
}

// ---------------------- Main read function -------------------------

float qmc_mag_read(void) {
    float mag_x, mag_y, mag_z;
    uint8_t Rec_Data[6];
    float angle;

    Rec_Data[0] = qmc_i2c_read(0x02); // X MSB
    Rec_Data[1] = qmc_i2c_read(0x01); // X LSB
    Rec_Data[2] = qmc_i2c_read(0x04); // Y MSB
    Rec_Data[3] = qmc_i2c_read(0x03); // Y LSB
    Rec_Data[4] = qmc_i2c_read(0x06); // Z MSB
    Rec_Data[5] = qmc_i2c_read(0x05); // Z LSB

    mag_x = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    mag_y = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    mag_z = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	

    // Basic tilt compensation (optional)
    float Xh = mag_x * cosf(mpu_accel_read(1)) + mag_z * sinf(mpu_accel_read(1));
    float Yh = mag_x * sinf(mpu_accel_read(0)) * sinf(mpu_accel_read(1))
             + mag_y * cosf(mpu_accel_read(0))
             - mag_z * sinf(mpu_accel_read(0)) * cosf(mpu_accel_read(1));

// try usig this 2 functions for tilt compensation 
	
    // tilt compensation 2
    float roll = mpu_accel_read(0)
    float pitch = mpu_accel_read(1)
	float Xh = ( mag_z*sinf(roll) - mag_y*cosf(roll));
	float Yh = ( mag_x*cosf(pitch)+mag_y*sinf(pitch)*sinf(roll)+mag_z*sinf(pitch)*cosf(roll) )
	final_yaw = atan2f(num,den)* (180.0f / 3.14159f); 

	// tilt compensation 3
	float roll = mpu_accel_read(0);
    float pitch = mpu_accel_read(1);
	float Xh = ( (mag_x * cosf(pitch)) + (mag_y*sinf(pitch)) - (mag_z*cosf(roll)*sinf(pitch)) );
	float Yh = ( (mag_y*cosf(roll)) + (mag_z*sin(roll)) );
	final_yaw = atan2f(-Yh,Xh) * (180.0f / 3.14159f);
	
    // Raw heading in degrees
    angle = atan2f(-mag_y, mag_x) * (180.0f / 3.14159f);
    if (angle < 0.0f)
        angle += 360.0f;

    // First reading → treat as 0°
    if (!offset_calibrated) {
        heading_offset = angle;
        offset_calibrated = 1;
        printf("Compass auto-zeroed. Heading offset = %.2f°\r\n", heading_offset);
    }

    // Apply zero-offset correction
    angle = calibrate_heading(angle, heading_offset);

    return angle * (3.14159f / 180.0f);  // return radians
}

// ---------------------- Initialization -----------------------------

void qmc_init(void) {
    qmc_i2c_write(0x0B, 0x08);  // Control 2
    qmc_i2c_write(0x0A, 0xCD);  // Continuous, 200Hz, full scale
    printf("QMC5883L initialized.\r\n");
}


