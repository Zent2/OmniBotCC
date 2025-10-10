/*
 * as5600_idf_config.c
 * Interactive AS5600 configuration and monitor for ESP-IDF
 * Runs entirely on default UART0 (use `idf.py monitor` to interact).
 *
 * Uses the as5600 component (as5600.h / as5600.c) you provided that
 * implements: as5600_i2c_init(...), as5600_adc_init(...),
 * as5600_read_adc(...), read_raw_angle(...), read_zmco_register(...),
 * read_status_register(...), read_conf_register(...), read_mang_register(...),
 * read_zpos_register(...), write_burn_register(...), write_conf_register(...),
 * write_mang_register(...), write_zpos_register(...).
 *
 * Place this file under `main/` (or a component) and add to CMakeLists.txt.
 * Build with `idf.py build` and run `idf.py monitor` to interact.
 *
 * Author: ChatGPT (adapted for Christian Campos project)
 * Date: 2025
 */

#define BURN_SETTING_CMD 0x40
#define BURN_ANGLE_CMD 0x80
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "driver/adc.h"
#include "esp_vfs_dev.h"  

#include "as5600.h" 

static const char *TAG = "AS5600_CFG";

// Change this if your analog input uses a different ADC channel
#ifndef AS5600_ADC_CHANNEL
#define AS5600_ADC_CHANNEL ADC1_CHANNEL_4 // GPIO32 by default; change if needed
#endif

#define UART_RX_TIMEOUT_MS 10000

//---------------------------Functions Declaration------------------------------
static void
periodic_timer_callback(void *arg); // Forward declaration

// I2C handles (created by as5600_i2c_init)
static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t as5600_dev = NULL;



// UART helpers: read a single non-whitespace char with optional timeout (ms).
// Returns -1 on timeout.
static int 
uart_read_char(uint32_t timeout_ms)
{
    uint8_t ch;
    int len = uart_read_bytes(UART_NUM_0, &ch, 1, pdMS_TO_TICKS(timeout_ms));
    if (len <= 0)
        return -1;
    return (int)ch;
}

// waitForYes: prints msg and waits for Y/S or N. timeout_ms==0 -> wait forever.
static bool 
waitForYes(const char *msg, uint32_t timeout_ms)
{
    printf("%s (Y/N): ", msg);
    fflush(stdout);

    uint32_t start = xTaskGetTickCount();
    while (1)
    {
        int c = uart_read_char(100);
        if (c > 0)
        {
            // skip whitespace
            while (isspace((unsigned char)c))
            {
                c = uart_read_char(100);
                if (c < 0)
                    break;
            }
            if (c < 0)
                continue; // got only whitespace
            char ch = (char)c;
            ch = toupper((unsigned char)ch);

            // flush rest of the line
            uint8_t tmp;
            while (uart_read_bytes(UART_NUM_0, &tmp, 1, pdMS_TO_TICKS(10)) > 0)
            {
                if (tmp == '\n') break;
            }

            if (ch == 'Y' || ch == 'S')
            {
                printf("%c\n", ch);
                return true;
            }
            else if (ch == 'N')
            {
                printf("%c\n", ch);
                return false;
            }
            else
            {
                printf("\nRespuesta no válida. Ingrese Y (sí) o N (no): ");
            }
        }

        if (timeout_ms > 0)
        {
            uint32_t now = xTaskGetTickCount();
            if ((now - start) >= pdMS_TO_TICKS(timeout_ms))
            {
                printf("\nTiempo de espera agotado.\n");
                return false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// The interactive configuration task 
static void interactive_config_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "Starting AS5600 interactive configuration over UART0");

    // Initialize I2C using the helper in your as5600.c
    as5600_i2c_init(&i2c_bus, &as5600_dev);

    // Initialize ADC channel used for the analog reading
    as5600_adc_init(AS5600_ADC_CHANNEL);

    vTaskDelay(pdMS_TO_TICKS(100));

    printf("\n=== Configuración AS5600 paso a paso (ESP-IDF) ===\n");
    printf("ATENCIÓN: Las operaciones BURN son irreversibles. "
           "Proceda con cuidado.\n\n");

    if (!waitForYes("¿Desea configurar el AS5600?", 15000))
    {
        printf("Se ha elegido no configurar el AS5600 (o timeout). "
               "Saltando configuración.\n\n");
        // fall through to the monitoring loop
    }
    else
    {
        uint8_t zmco = 0;

        // Paso 2: Leer STATUS_REG
        if (waitForYes("2) Leer STATUS_REG", 0))
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            uint8_t status = read_status_register(as5600_dev);
            if (status == 0xFF)
            {
                printf("Error leyendo STATUS_REG (I2C).\n");
            }
            else
            {
                bool md = (status & (1 << 5));
                printf("   MD (magnet detect): %s\n", md ? "OK" : "NO DETECTADO");
                while (!md)
                {
                    printf("   Coloca un imán adecuado y confirma nuevamente.\n");
                    while (!waitForYes("   ¿Seguir?", 0))
                    {
                        printf("   Esperando confirmación para continuar...\n");
                        vTaskDelay(pdMS_TO_TICKS(500));
                    }
                    status = read_status_register(as5600_dev);
                    if (status == 0xFF)
                    {
                        printf("Error leyendo STATUS_REG (I2C). Abortando paso.\n");
                        break;
                    }
                    md = (status & (1 << 5));
                    printf("   MD (magnet detect): %s\n", md ? "OK" : "NO DETECTADO");
                }
            }
        }
        else
        {
            printf("   Paso 2 saltado.\n");
        }

        // Paso 3: Leer ZMCO
        if (waitForYes("3) Leer ZMCO (veces BURN_ANGLE)", 0))
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            uint8_t r = read_zmco_register(as5600_dev);
            if (r == 0xFF)
            {
                printf("Error leyendo ZMCO (I2C).\n");
            }
            else
            {
                zmco = r & 0x03;
                printf("   ZMCO = %u\n", (unsigned)zmco);
            }
        }
        else
        {
            printf("   Paso 3 saltado.\n");
        }

        if (zmco == 0)
        {
            if (waitForYes("4) Leer CONF, aplicar ajustes y reescribir", 0))
            {
                vTaskDelay(pdMS_TO_TICKS(100));
                uint16_t conf = read_conf_register(as5600_dev);
                if (conf == 0xFFFF)
                {
                    printf("Error leyendo CONF (I2C).\n");
                }
                else
                {
                    // Apply the same bit masks as the Arduino sketch
                    conf &= ~(1 << 7);
                    conf &= ~(0b111 << 4);
                    conf = (conf & ~(0b11 << 2)) | (0b11 << 2);
                    conf &= ~(0b11);
                    if (write_conf_register(as5600_dev, conf) == ESP_OK)
                    {
                        printf("   CONF reescrito\n");
                    }
                    else
                    {
                        printf("   Error reescribiendo CONF\n");
                    }
                }

                if (waitForYes("5) Fijar MANG a 0x0FFF", 0))
                {
                    vTaskDelay(pdMS_TO_TICKS(100));
                    if (write_mang_register(as5600_dev, 0x0FFF) == ESP_OK)
                        printf("   MANG escrito\n");
                    else
                        printf("   Error escribiendo MANG\n");
                }
                else
                {
                    printf("   Paso 5 saltado.\n");
                }

                if (waitForYes("6) BURN_SETTING (0x40)", 0))
                {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    // write BURN_REG with BURN_SETTING_CMD using component helper
                    if (write_burn_register(as5600_dev, BURN_SETTING_CMD) == ESP_OK)
                    {
                        vTaskDelay(pdMS_TO_TICKS(100));
                        printf("   BURN_SETTING enviado\n");
                    }
                    else
                    {
                        printf("   Error enviando BURN_SETTING\n");
                    }
                }
                else
                {
                    printf("   Paso 6 saltado.\n");
                }

                if (waitForYes("7) Verificar: enviar 0x01,0x11,0x10 a BURN", 0))
                {
                    uint8_t seq[3] = {0x01, 0x11, 0x10};
                    write_burn_register(as5600_dev, seq[0]);
                    write_burn_register(as5600_dev, seq[1]);
                    write_burn_register(as5600_dev, seq[2]);
                    printf("   Secuencia enviada\n");
                }
                else
                {
                    printf("   Paso 7 saltado.\n");
                }
            }
            else
            {
                printf("   Pasos 4-7 saltados.\n");
            }
        }
        else
        {
            printf("No se puede configurar nuevamente el AS5600 (ZMCO != 0).\n");
        }

        // Paso 8 - leer MANG y CONF
        if (waitForYes("8) Leer MANG y CONF", 0))
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            uint16_t mang_check = read_mang_register(as5600_dev);
            uint16_t conf_check = read_conf_register(as5600_dev);
            if (mang_check == 0xFFFF || conf_check == 0xFFFF)
            {
                printf("Error leyendo MANG/CONF (I2C).\n");
            }
            else
            {
                printf("   MANG = 0x%04X\n", mang_check);
                printf("   CONF = 0x%04X\n", conf_check);
            }
        }
        else
        {
            printf("   Paso 8 saltado.\n");
        }

        // Pasos 9-13 si ZMCO < 3
        if (zmco < 3)
        {
            if (waitForYes("9) Leer RAW_ANGLE y fijar ZPOS", 0))
            {
                vTaskDelay(pdMS_TO_TICKS(100));
                uint16_t raw = (uint16_t)(read_raw_angle(as5600_dev) /
                                          360.0f * 4096.0f);
                // read_raw_angle returns float degrees; convert to raw 12-bit value
                if (raw == 0xFFFF)
                {
                    printf("Error leyendo RAW_ANGLE (I2C).\n");
                }
                else
                {
                    raw &= 0x0FFF;
                    if (write_zpos_register(as5600_dev, raw) == ESP_OK)
                    {
                        printf("   ZPOS fijado a 0x%04X\n", raw);
                    }
                    else
                    {
                        printf("   Error fijando ZPOS\n");
                    }
                }

                if (waitForYes("10) BURN_ANGLE (0x80)", 0))
                {
                    if (write_burn_register(as5600_dev, BURN_ANGLE_CMD) == ESP_OK)
                        printf("   BURN_ANGLE enviado\n");
                    else
                        printf("   Error enviando BURN_ANGLE\n");
                }
                else
                {
                    printf("   Paso 10 saltado.\n");
                }

                if (waitForYes("11) Verificar: enviar 0x01,0x11,0x10 a BURN", 0))
                {
                    uint8_t seq[3] = {0x01, 0x11, 0x10};
                    write_burn_register(as5600_dev, seq[0]);
                    write_burn_register(as5600_dev, seq[1]);
                    write_burn_register(as5600_dev, seq[2]);
                    printf("   Secuencia enviada\n");
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                else
                {
                    printf("   Paso 11 saltado.\n");
                }

                if (waitForYes("12) Leer ZPOS para comprobar", 0))
                {
                    uint16_t zpos_check = read_zpos_register(as5600_dev);
                    if (zpos_check == 0xFFFF)
                    {
                        printf("Error leyendo ZPOS (I2C).\n");
                    }
                    else
                    {
                        printf("   ZPOS = 0x%04X\n", zpos_check);
                    }
                }
                else
                {
                    printf("   Paso 12 saltado.\n");
                }
            }
            else
            {
                printf("   Pasos 9-12 saltados.\n");
            }

            if (waitForYes("13) Leer ZMCO final", 0))
            {
                vTaskDelay(pdMS_TO_TICKS(100));
                uint8_t zmco_final = read_zmco_register(as5600_dev);
                if (zmco_final == 0xFF)
                {
                    printf("Error leyendo ZMCO final (I2C).\n");
                }
                else
                {
                    zmco_final &= 0x03;
                    printf("   ZMCO final = %u\n", (unsigned)zmco_final);
                }
            }
            else
            {
                printf("   Paso 13 saltado.\n");
            }
        }
        else
        {
            printf("Tampoco se puede volver a configurar el ángulo 0 (ZMCO >= 3).\n");
        }

        printf("\n=== Configuración interactiva finalizada ===\n\n");
    }

    // After configuration, go into monitoring loop
    while (1)
    {
        float raw_angle_deg = read_raw_angle(as5600_dev);
        if (raw_angle_deg < 0.0f)
        {
            printf("Error leyendo RAW_ANGLE (I2C).\n");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        // Read ADC (using your as5600_read_adc helper)
        float analog_deg = as5600_read_adc(AS5600_ADC_CHANNEL, 10);

        printf("Lectura Digital (deg): %.2f\n", raw_angle_deg);
        printf("Angulo Analógico (ADC): %.2f deg\n", analog_deg);
        printf("----------------------------------------\n");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Never returns
}




void app_main(void)
{
    ESP_LOGI(TAG, "App starting. Use idf.py monitor to interact via UART0.");

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    esp_err_t ret;

    ret = uart_param_config(UART_NUM_0, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(ret));
        // decide si seguir o no; aquí salimos para evitar comportamiento indefinido
        return;
    }

    // reserva RX buffer (ej. 1024), tx buffer 0, sin cola de eventos
    ret = uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "uart_set_pin returned: %s", esp_err_to_name(ret));
        // no crítico: continuar
    }

    esp_vfs_dev_uart_use_driver(UART_NUM_0);

    // Lanzar la tarea interactiva
    BaseType_t xret = xTaskCreate(interactive_config_task, "as5600_cfg", 8192, NULL,
                tskIDLE_PRIORITY + 5, NULL);
    if (xret != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate failed (%d)", xret);
        // manejo adicional si quieres
    }
}

