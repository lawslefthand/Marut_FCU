/*
 * qmcr5883l.h
 *
 *  Created on: Oct 22, 2025
 *      Author: danba
 */

#ifndef INC_QMC5883L_H_
#define INC_QMC5883L_H_



// Define the I2C address for the QMC5883L
// The datasheet typically lists 0x0D as the 7-bit address.
#define QMC_ADDR 0x2C

/**
 * @brief Configures RCC clocks for I2C1 and GPIOB.
 */
void qmc_rcc_config(void);

/**
 * @brief Configures GPIOB pins (PB8, PB9) for I2C1 alternate function.
 */
void qmc_gpio_config(void);

/**
 * @brief Configures I2C1 peripheral for standard mode (100kHz).
 * @note Assumes a 16MHz APB1 clock (based on CCR and TRISE values).
 */
void qmc_i2c_config(void);

/**
 * @brief Writes a single byte to a specified register via I2C.
 * @param addr The register address to write to.
 * @param value The 8-bit value to write.
 */
void qmc_i2c_write(uint8_t addr, uint8_t value);

/**
 * @brief Reads a single byte from a specified register via I2C.
 * @param reg_addr The register address to read from.
 * @return The 8-bit value read from the register.
 */
uint8_t qmc_i2c_read(uint8_t reg_addr);

/**
 * @brief Reads raw magnetometer data (X, Y, Z) and calculates the heading.
 * @note This function prints the heading to the console (requires stdio.h).
 * It populates the global variables mag_x, mag_y, and mag_z.
 */
float qmc_mag_read(void);

/**
 * @brief Initializes the QMC5883L sensor.
 * Checks the chip ID and sets control registers for normal operation.
 */
void qmc_init(void);

float calibrate_compass(float current_heading);
float calibrate_heading(float current_heading,float heading_offset);



#endif /* INC_QMC5883L_H_ */
