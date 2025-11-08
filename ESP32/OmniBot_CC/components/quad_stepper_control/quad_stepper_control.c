/**
 * @file quad_stepper_control.c
 * @author Christian Campos (cam21760@uvg.com.gt)
 * @brief Library for configuring and controlling 4 stepper motor with DRV8825 
 * using ESP-IDF. The stepper motors are controlled using hardware timers
 * to achieve precise movement and speed control.
 * @version 1.0
 * @date 2025
 *
 * @copyright Copyright (c) 2025
 *
 */

//------------------------------External Libraries------------------------------
#include "driver/timer.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
//----------------------------Custom Libraries----------------------------------
#include "quad_stepper_control.h" // Library to handle stepper motor timers 

//------------------------------External Variables------------------------------
// These variables must be defined in the main code main.c and here as externs:
// Fixed variables
extern const gpio_num_t enable_pin; // Pin to enable/disable all motors
extern const gpio_num_t step_pins[4]; // STEP pins for 4 motors
extern const gpio_num_t dir_pins[4]; // Direction (DIR) pins for 4 motors 
extern const gpio_num_t ms_pins[3]; // Microstepping pins (MS1, MS2, MS3) 
extern const uint16_t steps_per_rev_base; // Base steps per revolution
// Global Variables
extern volatile uint8_t microstepping; // Microstepping (1, 2, 4, 8, 16, 32)
extern volatile uint16_t rpm[4]; // Rotation speed in RPM for each motor
extern volatile float rpm_error[4]; // Error in RPM for each motor
extern volatile bool active_motors[4]; // Motor state (active or not)
extern volatile bool direction_cw[4]; // Rotation direction for each motor 
extern TaskHandle_t hMotorControlTaskHandle;
// (true = CW, false = CCW) 

//------------------------------Internal Variables------------------------------
// These variables are internal to the timer module and should not be accessed 
// directly from the main code- They are defined here to avoid name conflicts 
// and maintain encapsulation
static uint64_t half_period_us[4] = {0};
static volatile bool  step_state[4]    = {false};
static const char *TAG_TIMER = "TIMER_STEPPER"; //TAG 
//--------------------------Definitions and Structures--------------------------
#define MOTOR_COUNT 4 // Number of motors
#define MICROSTEPPING_PINS 3 // Number of microstepping pins (MS1, MS2, MS3)
static timer_motor_map_t timer_map[4] = {
    { TIMER_GROUP_0, TIMER_0, 0 }, // Map motor 0 to timer 0 of group 0
    { TIMER_GROUP_0, TIMER_1, 1 }, // Map motor 1 to timer 1 of group 0
    { TIMER_GROUP_1, TIMER_0, 2 }, // Map motor 2 to timer 0 of group 1
    { TIMER_GROUP_1, TIMER_1, 3 } // Map motor 3 to timer 1 of group 1
};

//----------------------------Function Prototypes-------------------------------
//Local function prototypes
//Global function prototypes in header file quad_stepper_control.h
/**
 * @brief Task that controls the motors based on the current values of rpms, ms, etc.
 * @details This task runs on core 1 and is notified from other parts of the 
 * code when there are changes in the motor parameters.
 */
static void 
quad_motor_control_task(void *arg);

/**
 * @brief This function configures the GPIO pins to control stepper motors 
 * (MS, DIR, and STEP).
 * 
 * @param arg 
 */
static void 
timer_config_task(void *arg); // Task to configure timers

/**
 * @brief Configures a hardware timer to generate STEP pulses on a motor.
 * 
 * Initializes the timer, registers the ISR, and leaves the timer paused until 
 * the first start.
 * 
 * @param motorIndex Motor index (0–3).
 **/

static void 
configure_timer_for_motor(uint8_t motorIndex);

/**
* @brief This function configures the microstepping pins (MS1, MS2, MS3) 
according to the specified microstepping level. Should be called whenever 
the microstepping level is changed.
Parameters:
* @param ms_pins Array of microstepping pins.
* @param microstepping Microstepping level (1, 2, 4, 8, 16, 32)
* @return ESP_OK if configuration was successful, or an error code otherwise.
*/
static void  
quad_stepper_set_step(const gpio_num_t ms_pins[], uint8_t microstepping);

/**
 * @brief Updates the timer alarm value according to rpm and active state of 
 * the motor.
 * 
 * Pauses the timer, recalculates the half period, and if the motor is active,
 * restarts the timer to emit STEP pulses.
 * 
 * @param motorIndex Motor index (0–3).
 **/
static void 
update_timer_interval(uint8_t motorIndex, float error);

/**
 * @brief Calculates the half period (in µs) for the given motor, 
 * according to rpm and microstepping.
 * 
 * @param motorIndex Motor index (0–3).
 * @return Half period in microseconds, or 0 if rpm≤0.
 **/
static uint64_t 
calculate_half_period_us(uint8_t motorIndex, float error);

/**
 * @brief Timer interrupt service routine (generates toggling of the STEP pin).
 * 
 * @param arg Pointer to the timer→motor mapping structure.
 **/
static void 
IRAM_ATTR timer_isr(void *arg);


//--------------------------------Global Functions------------------------------

void 
quad_stepper_init()
{
    // Create a task to configure the timers on core 1
    xTaskCreatePinnedToCore(timer_config_task,
                            "timer_config_task", 2048, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(quad_motor_control_task, "quad_motor_control_task",
        4096, NULL, 5, &hMotorControlTaskHandle, 1);
    // ESP_LOGI(TAG_TIMER, "Timer configuration task created on core 1");
};

esp_err_t
quad_stepper_gpio_setup(const gpio_num_t dir_pins[], 
    const gpio_num_t step_pins[], const gpio_num_t ms_pins[], 
    volatile bool direction_cw[], uint8_t microstepping)
{

    for (int i = 0; i < MICROSTEPPING_PINS; i++)
    {
        // Configure microstepping pins
        //gpio_reset_pin(ms_pins[i]);
        gpio_set_direction(ms_pins[i], GPIO_MODE_OUTPUT);
        // Initialize the microstepping pins to low
        // gpio_set_level(ms_pins[i], 0);
    }
    // Configure microstepping pins (MS1, MS2, MS3)
    //quad_stepper_set_step(ms_pins, microstepping); //Disabled 
    // Configure direction and step pins for 4 motors
    gpio_reset_pin(GPIO_NUM_14); //This PIN is SPI CLK, so we reset it first
    gpio_reset_pin(GPIO_NUM_23);
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        //gpio_reset_pin(dir_pins[i]);
        //gpio_reset_pin(step_pins[i]);
        // Configure direction and step pins
        gpio_set_direction(dir_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_direction(step_pins[i], GPIO_MODE_OUTPUT);
        // Initialize the pins to low to avoid unwanted movements
        gpio_set_level(dir_pins[i], direction_cw[i] ? 0 : 1);
        gpio_set_level(step_pins[i], 0);
        // configure_timer_for_motor(i);
    }
    gpio_set_direction(enable_pin, GPIO_MODE_OUTPUT);
    //gpio_set_level(enable_pin, 1); // All motors disable (active low)
    // ESP_LOGI(TAG, "GPIO configured successfully");
    return ESP_OK;
}


//--------------------------------Local Functions-------------------------------
static void 
quad_motor_control_task(void *arg)
{
    // Variables to store the last values of:
    // RPM, error, direction, microstepping and active state of the motors
    static uint16_t last_rpm[4] = {0};
    static bool last_dir[4] = {false, false, false, false};
    static bool last_activo[4] = {false, false, false, false};
    static float last_rpm_error[4] = {0};
    static uint8_t last_microstepping = -1;
    static bool b_update_flag[4] = {false, false, false, false};

    // Flag to indicate if an update is needed
    // These allow checking if values have changed to avoid unnecessary updates
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // Wait for the task to be notified
        for (int i = 0; i < 4; i++)
        {

            if (rpm[i] != last_rpm[i])
            {
                last_rpm[i] = rpm[i];
                b_update_flag[i] = true;
            }

            if (active_motors[i] != last_activo[i])
            {
                last_activo[i] = active_motors[i];
                b_update_flag[i] = true;
            }

            if (rpm_error[i] != last_rpm_error[i])
            {
                last_rpm_error[i] = rpm_error[i];
                b_update_flag[i] = true;
            }

            if (direction_cw[i] != last_dir[i])
            {
                bool invert_dir[4] = {0, 1, 1, 0};
                last_dir[i] = direction_cw[i];
                bool dir = direction_cw[i] ^ invert_dir[i]; // Invert direction
                // for motors 1 and 2
                gpio_set_level(dir_pins[i], dir ? 0 : 1);
            }

            if (b_update_flag[i]) // If any value has changed, update the timer
            {
                update_timer_interval(i, last_rpm_error[i]);
                // Update the timer interval for the motor
                b_update_flag[i] = false; // Reset the update flag
            }
        }
    #if 0 
        if (microstepping != last_microstepping)
        {
            last_microstepping = microstepping; // Update the last microstepping
            // Set the microstepping pins according to the new value
            //quad_stepper_set_step(ms_pins, microstepping);
            for (int i = 0; i < 4; i++)
            {
                update_timer_interval(i, 0); // Update all the timers
            }

            if (quad_stepper_set_step(ms_pins, microstepping) != ESP_OK) 
            {
                ESP_LOGE("CONTROL_TASK", "Error setting microstepping: %d", 
                    microstepping);
            }

            else
            {
                ESP_LOGI("CONTROL_TASK", "Microstepping applied: %d", 
                    microstepping);
                for (int i = 0; i < 4; i++)
                    update_timer_interval(i,0); // Update all the timers 
            }

        }
    #endif
    }
    vTaskDelete(NULL); // Delete the task when it finishes
}

static void 
timer_config_task(void *arg)
{
    for (int i = 0; i < 4; i++)
    {
        configure_timer_for_motor(i);
    }

    vTaskDelete(NULL); // Delete the task after configuring the timers
}

// Configure a timer for each motor
static void 
configure_timer_for_motor(uint8_t motorIndex) 
{
    timer_motor_map_t *map = &timer_map[motorIndex]; // Get the motor mapping
    timer_config_t config = {
        .divider = 80,                 // 80 MHz/80 = 1 MHz → tick=1μs
        .counter_dir = TIMER_COUNT_UP, // Counter in up mode
        .counter_en = TIMER_PAUSE,     // Start the timer in pause
        .alarm_en = TIMER_ALARM_EN,    // Enable timer alarm
        .auto_reload = true,           // Enable auto-reload (automatically)
        .intr_type = TIMER_INTR_LEVEL, // Set level interrupt
    };
    // Initialize the timer with the configuration
    timer_init(map->group, map->timer, &config);
    timer_set_counter_value(map->group, map->timer, 0); // Reset the timer
    // Calculate the half period in microseconds (function defined above)
    half_period_us[motorIndex] = calculate_half_period_us(motorIndex, 0);
    // Set the timer alarm value; if half period is 0, use 1
    timer_set_alarm_value(map->group, map->timer,
                          half_period_us[motorIndex] == 0 ? 1 : 
                          half_period_us[motorIndex]);
    timer_enable_intr(map->group, map->timer); // Enable timer interrupt
    timer_isr_register(map->group, map->timer, // Register the timer ISR
                       timer_isr, (void *)map,
                       ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1 | 
                       ESP_INTR_CPU_AFFINITY_0, NULL);
}

static void 
quad_stepper_set_step(const gpio_num_t ms_pins[], uint8_t microstepping)
{
    bool mode0, mode1, mode2; // Variables for microstepping modes
    switch (microstepping)
    {
    case 1: // Full step (MODE2=0, MODE1=0, MODE0=0)
        mode0 = 0;
        mode1 = 0;
        mode2 = 0;
        break;

    case 2: // 1/2 step (MODE2=0, MODE1=0, MODE0=1)
        mode0 = 1;
        mode1 = 0;
        mode2 = 0;
        break;

    case 4: // 1/4 step (MODE2=0, MODE1=1, MODE0=0)
        mode0 = 0;
        mode1 = 1;
        mode2 = 0;
        break;

    case 8: // 8 microsteps/step (MODE2=0, MODE1=1, MODE0=1)
        mode0 = 1;
        mode1 = 1;
        mode2 = 0;
        break;

    case 16: // 16 microsteps/step (MODE2=1, MODE1=0, MODE0=0)
        mode0 = 0;
        mode1 = 0;
        mode2 = 1;
        break;

    case 32: // 32 microsteps/step. (All others options)
        mode0 = 1;
        mode1 = 1;
        mode2 = 1;
        break;

    default:
        // return ESP_ERR_INVALID_ARG;
        mode0 = 1;
        mode1 = 1;
        mode2 = 1;
    }

    // Set the microstepping pins based on the modes
    gpio_set_level(ms_pins[0], mode0); // MODE0
    gpio_set_level(ms_pins[1], mode1); // MODE1
    gpio_set_level(ms_pins[2], mode2); // MODE2

    // return ESP_OK;
}


static void 
update_timer_interval(uint8_t motorIndex, float error) // Update the timer
{
    timer_motor_map_t *map = &timer_map[motorIndex]; // Get the motor mapping
    // Calculate the new half period in microseconds
    uint64_t newHalf = calculate_half_period_us(motorIndex, error);
    // Update the half period in the internal variable
    half_period_us[motorIndex] = newHalf;
    // Pause the timer before updating the value
    timer_pause(map->group, map->timer);
    timer_set_counter_value(map->group, map->timer, 0); // Reset the timer

    if (newHalf == 0 || !active_motors[motorIndex])
    {
        gpio_set_level(step_pins[motorIndex], 0);
        //gpio_set_level(enable_pin, 1); // Disable all motors (active low)
        step_state[motorIndex] = false;
    }

    else
    {
        // Set the new timer alarm value
        timer_set_alarm_value(map->group, map->timer, newHalf);
        // If the motor is active, restart the timer to emit STEP pulses
        step_state[motorIndex] = false; // Reset the step state of the motor
        gpio_set_level(step_pins[motorIndex], 0); // Ensure the STEP pin is low
        //gpio_set_level(enable_pin, 0); // Enable all motors (active low)
        timer_start(map->group, map->timer);      // Start the timer
    }
}

static uint64_t
calculate_half_period_us(uint8_t motorIndex, float error)
{
    uint16_t spb = steps_per_rev_base; // Base steps per revolution
    uint8_t ms = microstepping;        // Microstepping level (1,2,4,8,16,32)
    double r = (double)rpm[motorIndex]; // RPM as double

    // Apply PID error
    r += (double)error;

    // Safety checks
    if (spb == 0 || ms == 0) {
        ESP_LOGW(TAG_TIMER, 
            "Invalid steps_per_rev_base (%u) or microstepping (%u).", 
            (unsigned)spb, (unsigned)ms);
        return 0;
    }
    // If r is NaN or <= 0, the comparison (r > 0.0) will be false -> stop motor
    if (!(r > 0.0)) {
        return 0;
    }

    double totalSteps = (double)spb * (double)ms;
    double denom = totalSteps * r; // steps * RPM

    // denom must be a positive finite value; 
    // (denom > 0.0) is false for NaN and negatives
    if (!(denom > 0.0)) {
        ESP_LOGW(TAG_TIMER, 
            "Invalid denominator in period calc: totalSteps=%.0f r=%.6f", 
            totalSteps, r);
        return 0;
    }

    double period_us = 60000000.0 / denom; // period in microseconds (floating)

    // period_us must be positive; comparison also filters out NaN
    if (!(period_us > 0.0)) {
        ESP_LOGW(TAG_TIMER, "Computed invalid period_us: %.6f", period_us);
        return 0;
    }

    uint64_t periodUs_u64 = (uint64_t)(period_us + 0.5); // round
    if (periodUs_u64 == 0) {
        return 0;
    }
    //ESP_LOGI(TAG_TIMER, "Period Set to: %llu us", periodUs_u64);
    return periodUs_u64 / 2; // Return the half period in microseconds
}


void
    IRAM_ATTR timer_isr(void *arg)
{
    timer_motor_map_t *map = (timer_motor_map_t *)arg; // Get the motor map
    timer_group_clr_intr_status_in_isr(map->group, map->timer); // Clear timer
    int idx = map->motor_idx; // Get the motor index from the map
    step_state[idx] = !step_state[idx]; // Toggle the step state of the motor
    gpio_set_level(step_pins[idx], step_state[idx]); // Update the STEP pin
    timer_group_enable_alarm_in_isr(map->group, map->timer); // Enable alarm
}
