idf_component_register(SRCS "udp_wifi.c"
                    INCLUDE_DIRS "include"
                    REQUIRES  "driver" "esp_wifi" "nvs_flash" 
                    )
