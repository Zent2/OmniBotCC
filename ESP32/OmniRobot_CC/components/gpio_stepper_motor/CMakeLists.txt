# components/gpio_setup/CMakeLists.txt
idf_component_register(
    SRCS "gpio_stepper_motor.c"
    INCLUDE_DIRS "include"
    REQUIRES  "driver" 
    )
