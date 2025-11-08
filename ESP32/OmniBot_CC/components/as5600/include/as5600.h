/**
 * @file as5600.c
 * @author Christian Campos (cam21760@uvg.edu.gt)
 * @brief This file contains the implementation of the AS5600 encoder. 
 * It provides a function to read the ADC value from the specified channel
 * and average it over a number of samples, returning the angle in degrees.
 * Also I2C communication 
 * @version 0.1
 * @date 2025-06-24
 * 
 * @copyright Copyright (c) 2025
 * @todo Add I2C communication for AS5600 encoder
 */
#ifndef AS5600_H
#define AS5600_H
//------------------------------------------------------------------------------
//------------------------------External Libraries------------------------------
#include "driver/adc.h"
#include "driver/i2c.h"
#include "driver/i2c_master.h"

//-----------------------------Function Prototypes------------------------------
/**
 * @brief Initializes the ADC for the AS5600 encoder by configuring the ADC 
 * channel width and attenuation. (12 bits width and 11dB attenuation).
 * 
 * @param channel 
 */
void 
as5600_adc_init(adc1_channel_t channel);
/**
 * @brief Reads the ADC value from the specified channel and averages it over a 
 * number of samples.
 * 
 * @param channel ADC1 channel to read from (e.g., ADC1_CHANNEL_0).
 * @param samples Number of samples to average (must be greater than 0).
 * @return float The averaged ADC value mapped to an angle in degrees [0, 360).
 */
float 
as5600_read_adc(adc1_channel_t channel, uint8_t samples);

//-------------------------------I2C Functions----------------------------------

/**
 * @brief Initializes the I2C bus and device handle for the AS5600 encoder.
 * @param bus_handle Pointer to the I2C bus handle.
 * @param dev_handle Pointer to the I2C device handle.
 * @param address I2C address of the AS5600 encoder (default is 0x36).
 */

void 
as5600_i2c_init(i2c_master_bus_handle_t *bus_handle, 
    i2c_master_dev_handle_t *dev_handle);

/**
 * @brief Reads from the AS5600 I2C register.
 * 
 * @param dev_handle I2C device handle for the AS5600 encoder.
 * @param reg_addr Address of the register to read from.
 * @param data Pointer to the buffer to store the read data.
 * @param len Length of the data to read.
 */
static esp_err_t
read_as5600_register(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, 
    uint8_t *data, size_t len);

/**
 * @brief Writes to the AS5600 I2C register.
 * 
 * @param dev_handle I2C device handle for the AS5600 encoder.
 * @param reg_addr Address of the register to write to.
 * @param data Data byte to write.
 */
static esp_err_t
write_as5600_register(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, 
    const uint8_t *data, size_t len);

/**
 * @brief Reads the ZMCO_REG register.
 * 
 * @param dev_handle I2C device handle for the AS5600 encoder.
 * @return uint8_t Value read from the ZMCO_REG register.
 */

uint8_t
read_zmco_register(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Reads the ZPOS_REG register.
 * 
 * @param dev_handle I2C device handle for the AS5600 encoder.
 * @return uint16_t Value read from the ZPOS_REG register.
 */
uint16_t
read_zpos_register(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Reads the MANG_REG register.
 * 
 * @param dev_handle I2C device handle for the AS5600 encoder.
 * @return uint16_t Value read from the MANG_REG register.
 */
uint16_t 
read_mang_register(i2c_master_dev_handle_t dev_handle);

/** 
 * @brief Reads the CONF_REG register.
 * 
 * @param dev_handle I2C device handle for the AS5600 encoder.
 * @return uint16_t Value read from the CONF_REG register.
 */
uint16_t
read_conf_register(i2c_master_dev_handle_t dev_handle);

/** 
 * @brief Reads the STATUS_REG register.
 * 
 * @param dev_handle I2C device handle for the AS5600 encoder.
 * @return uint8_t Value read from the STATUS_REG register.
 */

uint8_t
read_status_register(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Reads the RAW_ANGLE register.
 * 
 * @param dev_handle I2C device handle for the AS5600 encoder.
 * @return float Raw angle value read from the RAW_ANGLE register.
 */
float
read_raw_angle(i2c_master_dev_handle_t dev_handle);


/**
 * @brief Writes to the BURN_REG register.
 * 
 * @param dev_handle I2C device handle for the AS5600 encoder.
 * @param command Command byte to write to the BURN_REG register.
 */
esp_err_t
write_burn_register(i2c_master_dev_handle_t dev_handle, uint8_t command);

/**
 * @brief Writes a 16-bit value to the ZPOS register.
 * 
 * @param dev_handle I2C device handle for the AS5600 encoder.
 * @param value 16-bit value to write to the ZPOS register.
 * @return esp_err_t 
 */

esp_err_t 
write_zpos_register(i2c_master_dev_handle_t dev_handle, uint16_t value);

/**
 * @brief Writes a 16-bit value to the MANG register.
 * 
 * @param dev_handle I2C device handle for the AS5600 encoder.
 * @param value 16-bit value to write to the MANG register.
 * @return esp_err_t 
 */
esp_err_t 
write_mang_register(i2c_master_dev_handle_t dev_handle, uint16_t value);

/**
 * @brief Writes a 16-bit value to the CONF register.
 * 
 * @param dev_handle I2C device handle for the AS5600 encoder.
 * @param value 16-bit value to write to the CONF register.
 * @return esp_err_t 
 */
esp_err_t 
write_conf_register(i2c_master_dev_handle_t dev_handle, uint16_t value);


#endif // AS5600_H
