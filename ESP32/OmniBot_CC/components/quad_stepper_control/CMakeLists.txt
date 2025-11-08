# components/timer_manager/CMakeLists.txt
idf_component_register(SRCS "quad_stepper_control.c"
                    INCLUDE_DIRS "include"
                    REQUIRES  "driver" )
