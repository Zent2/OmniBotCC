/**
 * @file as5600.c
 * @author Christian Campos (cam21760@uvg.edu.gt)
 * @brief This file contains the implementation of the AS5600 encoder reading function.
 * @version 0.1
 * @date 2025-06-24
 *
 * @copyright Copyright (c) 2025
 *
 */
//-------------------------------Custom Libraries-------------------------------
#include "as5600.h"
//------------------------------External Libraries------------------------------
#include "driver/adc.h" // ADC library for reading analog values
#include "driver/i2c.h"
#include "driver/i2c_master.h"

#define AS5600_ADDR 0x36 // I2C address of the AS5600 encoder
#define I2C_MASTER_TIMEOUT_MS 1000 // Timeout for I2C operations in milliseconds
// AS5600 registers
#define ZMCO_REG 0x00
#define ZPOS_REG_MSB 0x01
#define ZPOS_REG_LSB 0x02
#define MANG_REG_MSB 0x05
#define MANG_REG_LSB 0x06
#define CONF_REG_MSB 0x07
#define CONF_REG_LSB 0x08
#define STATUS_REG 0x0B
#define RAW_ANGLE_MSB 0x0C
#define RAW_ANGLE_LSB 0x0D
#define BURN_REG 0xFF

// BURN commands
#define BURN_SETTING_CMD 0x40
#define BURN_ANGLE_CMD 0x80

//----------------------------------Functions-----------------------------------

void as5600_adc_init(adc1_channel_t channel)
{
    // Set ADC width to 12 bits (0–4095)
    adc1_config_width(ADC_WIDTH_BIT_12);

    // Set attenuation to 11dB (~0–3.6V range, safe for 3.3V signals)
    adc1_config_channel_atten(channel, ADC_ATTEN_DB_11);
}

float as5600_read_adc(adc1_channel_t channel, uint8_t samples)
{
    if (samples <= 0)
    {
        samples = 1; // Ensure at least one sample is taken
    }
    else if (samples > 255)
    {
        samples = 255; // Limit the maximum number of samples to avoid overflow
    }

    uint32_t sum = 0.0f; // 255*4095 = 1036800, which fits in uint32_t
    for (uint8_t s = 0; s < samples; s++)
    {
        sum += adc1_get_raw(channel);
    }

    float adc_val = (float)sum / samples;

    // Map [0…4095] → [0…360)
    float current_angle = (adc_val / 4096.0f) * 360.0f;
    if (current_angle == 360.0f)
    {
        current_angle = 0.0f; // Wrap around to 0 if it reaches 360
    }

    return current_angle;
}

//-------------------------------I2C Functions----------------------------------

static void 
as5600_i2c_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    // Initialize the I2C bus
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    //Initialize the I2C handle
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
    //ESP_LOGI("AS5600", "I2C bus initialized for AS5600 encoder");
}
// Read a byte from the AS5600 I2C register
static esp_err_t
read_as5600_register(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

// Write a byte to the AS5600 I2C register 
static esp_err_t 
write_as5600_register(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data}; // The packet must have an address followed by the data
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

// Read the ZMCO_REG register
static uint8_t
read_zmco_register(i2c_master_dev_handle_t dev_handle)
{
    uint8_t zmco_data;
    read_as5600_register(dev_handle, ZMCO_REG, &zmco_data, 1);
    return zmco_data;
}

// Read the ZPOS_REG register
static uint16_t
read_zpos_register(i2c_master_dev_handle_t dev_handle)
{
    uint8_t zpos_data[2] = {0};
    read_as5600_register(dev_handle, ZPOS_REG_MSB, zpos_data, sizeof(zpos_data));
    return ((uint16_t)((zpos_data[0] << 8) | zpos_data[1]));
}

// Read the MANG_REG register
static uint16_t 
read_mang_register(i2c_master_dev_handle_t dev_handle)
{
    uint8_t mang_data[2] = {0};
    read_as5600_register(dev_handle, MANG_REG_MSB, mang_data, sizeof(mang_data));
    return ((uint16_t)((mang_data[0] << 8) | mang_data[1]));
}

// Read the CONF_REG register
static uint16_t
read_conf_register(i2c_master_dev_handle_t dev_handle)
{
    uint8_t conf_data[2] = {0};
    read_as5600_register(dev_handle, CONF_REG_MSB, conf_data, sizeof(conf_data));
    return ((uint16_t)((conf_data[0] << 8) | conf_data[1]));
}

// Read the STATUS_REG register
static uint8_t
read_status_register(i2c_master_dev_handle_t dev_handle)
{
    uint8_t status = 0;
    read_as5600_register(dev_handle, STATUS_REG, &status, sizeof(status));
    return status;
}
// Read the RAW_ANGLE register
static float
read_raw_angle(i2c_master_dev_handle_t dev_handle)
{
    uint8_t raw_angle_data[2] = {0};
    read_as5600_register(dev_handle, RAW_ANGLE_MSB, raw_angle_data, sizeof(raw_angle_data));
    return ((float)((raw_angle_data[0] << 8) | raw_angle_data[1])) / 4096.0f * 360.0f;

}


// Write to the BURN_REG register
static void
write_burn_register(i2c_master_dev_handle_t dev_handle, uint8_t command)
{
    write_as5600_register(dev_handle, BURN_REG, command);
}
