idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES spi_flash driver 
                       nvs_flash esp_timer quad_stepper_control 
                       process_command uart_commands udp_wifi 
                       as5600 gpio_stepper_motor)
