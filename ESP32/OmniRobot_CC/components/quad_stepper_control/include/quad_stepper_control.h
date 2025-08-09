
/**
 * @file quad_stepper_control.h
 * @author Christian Campos (cam21760@uvg.com.gt)
 * @brief Library for configuring and controlling 4 stepper motor with DRV8825 
 * using ESP-IDF. The stepper motors are controlled using hardware timers
 * to achieve precise movement and speed control.
 * @version 0.1
 * @date 2025
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef quad_stepper_control_H
#define quad_stepper_control_H

//--------------------------Definitions and Structures--------------------------

typedef struct { // Structure to map a timer to a motor
    timer_group_t group; // Timer group (0 or 1)
    timer_idx_t   timer; // Timer index within the group (0 or 1)
    int           motor_idx; // Motor index associated with this timer (0–3)
} timer_motor_map_t;


//-----------------------------Function Prototypes------------------------------

/**
 * @brief This function configures the GPIO pins to control stepper motors (MS, DIR, and STEP).
 * Should be called before starting the motors and only once.
 *
 * Compatible with stepper motor drivers such as A4988, DRV8825, TMC2209.
 *
 * @param dir_pins Array of direction pins for the motors.
 * @param step_pins Array of step pins for the motors.
 * @param ms_pins Array of microstepping pins.
 * @param direction_cw Array of booleans indicating the rotation direction (CW or CCW) for each motor.
 * @param microstepping Microstepping level (1, 2, 4, 8, 16, 32).
 * @return ESP_OK if configuration was successful, or an error code otherwise.
 */
esp_err_t 
quad_stepper_gpio_setup(const gpio_num_t dir_pins[], const gpio_num_t step_pins[], 
    const gpio_num_t ms_pins[], volatile bool direction_cw[], uint8_t microstepping);


/**
 * @brief Initializes the timers for the stepper motors.
 * This function creates a task that configures the timers on core 1.
 */
void 
quad_stepper_init();

#endif // quad_stepper_control_H