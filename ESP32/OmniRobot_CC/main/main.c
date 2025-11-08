// Project name: Robotat Omnidirectional Agent (OmniBotCC)
/**
 * @file main.c
 * @brief Main application entry point for ESP-IDF project. Initializes
 * peripherals, configures Wi-Fi, ADC, GPIOs, timers, and handles command
 * parsing and control loop. Uses PID for closed-loop control of stepper motors
 * with AS5600 encoders.
 * @note This project is part of a graduation thesis focusing on control of an
 * omnidirectional robotic agent using a ESP32 microcontroller, inside the
 * Robotat ecosystem of Universidad del Valle de Guatemala. The agent controls
 * four stepper motors to move in a 2D plane, and uses UDP for communication
 * with the Robotat ecosystem.
 * @note The stepper motors are controlled using timers to achieve precise
 * movement and speed control. Also, the stepper motors are equipped with AS5600
 * encoders to measure their position and calculate speed (closed Loop PID 
 * control).
 * @note This project uses the C17 language standard version.
 * @author Christian Campos
 * @date 2025
 * @version 1.0
 */
/* Max line length: 80 characters.
 * Ensure each source line does not exceed 80 chars.
 */
//------------------------------------------------------------------------------
//------------------------------External Libraries------------------------------

#include "freertos/FreeRTOS.h" // FreeRTOS (for tasks and timers)
#include "freertos/task.h"     // FreeRTOS library (for tasks)
#include "driver/gpio.h"       // GPIO library (for pin configuration)
#include "driver/uart.h"       // UART library (for serial communication)
#include "driver/timer.h"      // Timer library (for handling timers)
#include "esp_log.h"           // Logging library (for debug messages)
#include "nvs_flash.h"
#include "esp_err.h"

//------------------------------Standard C libraries----------------------------

#include <string.h> // String library (for handling strings)
#include <stdlib.h> // Standard library (for conversion and memory management)
#include <stdint.h> // Fixed-width integer types

//------------------------------ESP-IDF Libraries-------------------------------
// ESP_Timer libraries
#include <stdio.h>     // Standard input/output library (for printf and similar)
#include <unistd.h>    // Unix library (for sleep and other POSIX functions)
#include "esp_timer.h" // ESP-IDF timer library (for high-resolution timers)
#include "esp_sleep.h" // ESP-IDF deep sleep library (for low power mode)
#include "sdkconfig.h" // ESP-IDF SDK configuration (for project configuration)

// ADC libraries
#include "driver/adc.h" // ADC library (for ADC converter handling)

//------------------------------Custom Libraries--------------------------------
// #include "gpio_stepper_motor.h" // GPIO configuration for steppers 
#include "quad_stepper_control.h" // Library for stepper motor control
#include "udp_wifi.h"             // Library for WiFi configuration 
#include "as5600.h"               // Library for AS5600 encoder handling
// static const char *TAG = "TIMER_STEPPER"; // TAG name for logging

#define CONFIG_ESP_TIMER_TASK_AFFINITY 1 // CPU core affinity for ESP timer task
//-------------------------------Pin Definitions--------------------------------

// quad_stepper_control pins
// Stepper motors
// MS1,MS2,MS3
const gpio_num_t ms_pins[3] = {GPIO_NUM_NC, GPIO_NUM_NC, GPIO_NUM_NC};
//MS1, MS2 and MS3 not used (Set to 32 microstepping directly)
//Enable pin (shared for all motors)
const gpio_num_t enable_pin=GPIO_NUM_23; // Pin to enable/disable all motors
// Motor pins
const gpio_num_t step_pins[4] = {
    GPIO_NUM_26, GPIO_NUM_19, GPIO_NUM_16, GPIO_NUM_14};
const gpio_num_t dir_pins[4] = {
    GPIO_NUM_25, GPIO_NUM_18, GPIO_NUM_4, GPIO_NUM_27};
// AS5600 pins (ADC channels for reading angles)
// ADC1 channels for AS5600 encoders
const gpio_num_t as5600_pins[4] = {
    GPIO_NUM_34, GPIO_NUM_35, GPIO_NUM_32, GPIO_NUM_33};
const adc1_channel_t channels[4] = {
    ADC1_CHANNEL_6, // GPIO34
    ADC1_CHANNEL_7, // GPIO35
    ADC1_CHANNEL_4, // GPIO32
    ADC1_CHANNEL_5  // GPIO33
};
// STEP and DIR pin configuration for 4 motors
// Pins 32-33 motor 0
// Pins 19-18 motor 1
// Pins 27-14 motor 2
// Pins 16-4  motor 3

//---------------------------Parameters and Variables---------------------------
// Wifi
const uint16_t udp_port = 12345;         // UDP port for receiving commands
const gpio_num_t led_wifi = GPIO_NUM_2; // LED pin to indicate WiFi status

// quad_stepper_control parameters and variables
//  Motor parameters
const uint16_t steps_per_rev_base = 200; // Steps per revolution (Datasheet)
const uint16_t rpm_max = 400; // Maximum allowed RPM for the motors

// Volatile variables for motor control
//(this variables are shared between tasks and ISR)
volatile uint8_t microstepping = 32;         // ustepping level 
volatile uint16_t rpm[4] = {10, 10, 10, 10}; // Rotation speed in RPM
volatile float rpm_error[4] = {0, 0, 0, 0};  // Error in RPM
volatile bool direction_cw[4] = {true, false, false, true};
// Rotation direction for each motor (true = CW, false = CCW)
volatile bool active_motors[4] = {false, false, false, false};
// Motor state (active or not)

// PID control variables and parameters (for speed control with AS5600)
const int32_t esp_timer_period_us = 50000; // Timer period in microseconds
// 50000 us = 50 ms = 20 Hz (20 times per second)
const uint8_t samples = 15;             // Number of samples to average
volatile float last_angle[4] = {0.00f}; // Previous angle per motor
// volatile int64_t last_time_us[4] = {0};
volatile uint8_t print_counter = 0; // Counter to print every 2s
volatile float integral_error[4] = {0.00f};
volatile float prev_error[4] = {0.00f};

// PID parameters (tuning required)
volatile float Kp = 0.04;
volatile float Ki = 0.002;
volatile float Kd = 0.0002;
TaskHandle_t hMotorControlTaskHandle = NULL;
// New task for PID
TaskHandle_t hPIDTaskHandle = NULL;

static const char *TAG = "APP_MAIN";

//-------------------------Structures and Definitions---------------------------

//--------------------------------Command Queue---------------------------------
#define CMD_QUEUE_LENGTH 10
#define CMD_BUFFER_SIZE 128
static QueueHandle_t command_queue;

//---------------------------Functions Declaration------------------------------
static void
periodic_timer_callback(void *arg); // Forward declaration

static void
pid_task(void *arg); // Forward declaration

static void
process_command(const char *buf); // Forward declaration

void my_message_handler(const uint8_t *data, uint32_t len,
                        const struct sockaddr *src_addr,
                        void *arg); // Forward declaration

//----------------------------Auxiliary Functions-------------------------------

void init(void)
{
    // GPIO configuration (Dir, Step and MS)
    if (quad_stepper_gpio_setup(dir_pins, step_pins, ms_pins, direction_cw,
                                microstepping) != ESP_OK)
    {
        ESP_LOGE(TAG, "Pin configuration failed");
    }
    else
    {
        ESP_LOGI(TAG, "Pins configured correctly");
    }

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if ((ret == ESP_ERR_NVS_NO_FREE_PAGES) ||
        (ret == ESP_ERR_NVS_NEW_VERSION_FOUND))
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    // Connect to WiFi (with static IP and LED on)
    udp_connect_wifi(true, true);
    // Start UDP server with my handler
    udp_server_init(my_message_handler, NULL);

    // Timer configuration for each motor

    quad_stepper_init(); // Initialize the motor timers

    // Initialize the ESP_TIMER
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        .name = "AS5600_Timer",
    };

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));

    /* Start the timers */
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 
        esp_timer_period_us));

    // AS5600 ADC configuration
    for (int32_t i = 0; i < 4; i++)
    {
        gpio_reset_pin(as5600_pins[i]);
        gpio_set_direction(as5600_pins[i], GPIO_MODE_INPUT);
        as5600_adc_init(channels[i]);
        last_angle[i] = as5600_read_adc(channels[i], samples);
        
    }
    gpio_set_level(enable_pin, 1); // All motors disable (active low)
    // Create PID task
    xTaskCreatePinnedToCore(pid_task, "pid_task", 4096,
                            NULL, 6, &hPIDTaskHandle, 1);
}

// periodic_timer_callback: calculates RPM and notifies PID task
static void
periodic_timer_callback(void *arg)
{
    (void)arg;
    static const float Ts = esp_timer_period_us / 1000000.0f; // in seconds
    static const int32_t printing = 1000000 / esp_timer_period_us; // every 1s
    for (int32_t i = 0; i < 4; i++)
    {
        if (active_motors[i] && rpm[i] > 0)
        {
            // 1) Read the angle from the AS5600 encoder
            float current_angle = as5600_read_adc(channels[i], samples);

            // Determine if both angles are inside the "valid" window [20, 270]
            bool last_in_window = 
            (last_angle[i] >= 20.0f) && (last_angle[i] <= 270.0f);
            bool curr_in_window = 
            (current_angle >= 20.0f) && (current_angle <= 270.0f);

            if (last_in_window && curr_in_window)
            {
                // 2) "Signed" calculation according to direction:
                //    CW: Δ = current - last
                //   CCW: Δ = last    - current
                float delta = direction_cw[i]
                                  ? (current_angle - last_angle[i])
                                  : (last_angle[i] - current_angle);

                if (delta < 0.0f)
                {
                    delta += 360.0f;
                }

                float rpm_calculated = 0.0f;
                if (Ts > 0.0f)
                {
                    rpm_calculated = (delta / 360.0f) * (60.0f / Ts);
                }
                else
                {
                    rpm_calculated = 0.0f;
                }

                if (!(rpm_calculated >= 0.0f))
                {
                    rpm_calculated = 0.0f;
                }
                else if (rpm_calculated > 1.25f * (float)rpm[i])
                {
                    rpm_calculated = 1.25f * (float)rpm[i];
                }
                else if (rpm_calculated < 0.75f * (float)rpm[i])
                {
                    rpm_calculated = 0.75f * (float)rpm[i];
                }

                // Store measured RPM for PID task
                rpm_error[i] = rpm_calculated;

                // Print info every "printing" cycles (disabled by default)
                #if 0
                if (print_counter >= (uint8_t)(printing - 1))
                {
                    ESP_LOGI(TAG,
                             "Motor %d: CurrentAngle: %.2f°, δθ=%.2f°, 
                             Ts=%.3fs, RPM_obj=%d, RPM_med=%.2f (calculated)",
                             (int)i, current_angle, delta, Ts, (int)rpm[i], 
                             rpm_calculated);
                }
                #endif
            }
            else
            {
                rpm_error[i] = 0.0f;
                #if 0
                if (print_counter >= (uint8_t)(printing - 1))
                {
                    ESP_LOGI(TAG, 
                    "Motor %d: RPM skipped (angles out of [20,270]). last=%.2f°
                    , curr=%.2f°", (int)i, last_angle[i], current_angle);
                }
                #endif
            }
            last_angle[i] = current_angle;
        }
    }

    print_counter++;
    if (print_counter >= (uint8_t)printing)
    {
        print_counter = 0;
    }

    // Notifies the PID task
    xTaskNotifyGive(hPIDTaskHandle);
}



static void
pid_task(void *arg)
{
    static const float Ts = esp_timer_period_us / 1000000.0f; // in seconds
    while (1)
    {
        // Wait for notification from timer callback
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 

        for (int32_t i = 0; i < 4; i++)
        {
            if (active_motors[i] && rpm[i] > 0)
            {
                float rpm_calculated = rpm_error[i]; 
                float error = rpm[i] - rpm_calculated;
                float P = Kp * error;

                // Anti-windup: Clamp integral_error[i] to avoid excessive 
                //accumulation
                integral_error[i] += error * Ts;
                if (integral_error[i] > 1.5 * rpm[i]) 
                {
                    integral_error[i] = 1.5 * rpm[i];
                }
                else if (integral_error[i] < -1.5 * rpm[i]) 
                {
                    integral_error[i] = -1.5 * rpm[i];
                }

                float derivative = 0.0f;
                if (Ts > 0.0f)
                {
                    derivative = (error - prev_error[i]) / Ts;
                }

                prev_error[i] = error;

                float control_signal = 
                P + Ki * integral_error[i] + Kd * derivative;

                if ((control_signal < (0.05f * rpm[i])) && 
                (control_signal > -(0.05f * rpm[i])))
                {
                    control_signal = 0.0f;
                }

                // rpm_error now stores the control signal for the motor
                // control task
                rpm_error[i] = control_signal;
            }
            else
            {
                rpm_error[i] = 0.0f;
            }
        }

        // Notifies the motor control task
        xTaskNotifyGive(hMotorControlTaskHandle);
    }
}

static void
process_command(const char *buf)
{
    ESP_LOGI(TAG, "Command received: %s", buf);

    #if 0
    if (strncmp(buf, "step=", 5) == 0)
    {
        int32_t mode = atoi(buf + 5);
        if ((mode == 8) || (mode == 32))
        {
            microstepping = (uint8_t)mode;
            ESP_LOGI(TAG, "Setting microstepping to: %d", (int)microstepping);
        }
        else
        {
            ESP_LOGW(TAG, "Invalid microstepping: %d", (int)mode);
        }
    }
    #endif
    
    if (strcmp(buf, "start") == 0)
    {
        for (int32_t i = 0; i < 4; i++)
        {
            active_motors[i] = true;
            gpio_set_level(enable_pin, 0);
            // update_timer_interval(i);
        }
        ESP_LOGI(TAG, "All motors started");
    }
    else if (strcmp(buf, "stop") == 0)
    {
        for (int32_t i = 0; i < 4; i++)
        {
            active_motors[i] = false;
            gpio_set_level(enable_pin, 1);
            // update_timer_interval(i);
        }
        ESP_LOGI(TAG, "All motors stopped");
    }   
    else
    {
        for (int32_t i = 0; i < 4; i++)
        {
            char prefix[16];
            snprintf(prefix, sizeof(prefix), "m%d_rpm=", (int)i);
            if (strncmp(buf, prefix, strlen(prefix)) == 0)
            {
                int32_t value = atoi(buf + strlen(prefix));
                if (value < 0)
                {
                    value = 0;
                }
                if (value > rpm_max)
                {
                    ESP_LOGW(TAG, "RPM exceeds limit (%d): %d", 
                        (int)rpm_max, (int)value);
                    value = rpm_max;
                }
                rpm[i] = (uint16_t)value;
                ESP_LOGI(TAG, "Motor %d RPM updated to: %d", 
                    (int)i, (int)rpm[i]);
                // update_timer_interval(i);
                break;
            }
            snprintf(prefix, sizeof(prefix), "m%d_dir=", (int)i);
            if (strncmp(buf, prefix, strlen(prefix)) == 0)
            {
                const char *d = buf + strlen(prefix);
                if (strcasecmp(d, "CW") == 0)
                {
                    direction_cw[i] = true;
                    // gpio_set_level(dir_pins[i], 0);
                    ESP_LOGI(TAG, "Motor %d direction: CW", (int)i);
                }
                else if (strcasecmp(d, "CCW") == 0)
                {
                    direction_cw[i] = false;
                    // gpio_set_level(dir_pins[i], 1);
                    ESP_LOGI(TAG, "Motor %d direction: CCW", (int)i);
                }
                else
                {
                    ESP_LOGW(TAG, "Invalid direction for motor %d: %s", 
                        (int)i, d);
                }
                break;
            }
            snprintf(prefix, sizeof(prefix), "m%d_start", (int)i);
            if (strcmp(buf, prefix) == 0)
            {
                active_motors[i] = true;
                // update_timer_interval(i);
                ESP_LOGI(TAG, "Motor %d started", (int)i);
                break;
            }
            snprintf(prefix, sizeof(prefix), "m%d_stop", (int)i);
            if (strcmp(buf, prefix) == 0)
            {
                active_motors[i] = false;
                // update_timer_interval(i);
                ESP_LOGI(TAG, "Motor %d stopped", (int)i);
                break;
            }
        }
    }
    if (hMotorControlTaskHandle != NULL)
    {
        xTaskNotifyGive(hMotorControlTaskHandle);
    }
}


// This function is called when a message is received via UDP
void my_message_handler(const uint8_t *data, uint32_t len,
                        const struct sockaddr *src_addr,
                        void *arg)
{
    char buf[128];
    if (len >= (int32_t)sizeof(buf))
        len = (int32_t)sizeof(buf) - 1;
    memcpy(buf, data, len);
    buf[len] = '\0';
    process_command(buf);
}

//---------------------------------Interrupts-----------------------------------

//-----------------------------------Tasks--------------------------------------

//Uncomment this function if UART task is needed
#if 0
void 
uart_task(void *arg)
{
    char buf[128];
    int32_t idx = 0;
    while (1)
    {
        uint8_t c;
        int32_t len = uart_read_bytes(UART_NUM_0, &c, 1, pdMS_TO_TICKS(10));
        if (len > 0)
        {
            if ((c == '\n') || (c == '\r'))
            {
                if (idx > 0)
                {
                    buf[idx] = '\0';
                    process_command(buf);
                    idx = 0;
                }
            }
            else if (idx < (int32_t)sizeof(buf) - 1)
            {
                buf[idx++] = c;
            }
        }
    }
}
#endif
//-------------------------------Main Function----------------------------------
void app_main(void)
{
    init(); 
    // System startup message
    ESP_LOGI(TAG, "System initialized. Waiting for commands...");
}

