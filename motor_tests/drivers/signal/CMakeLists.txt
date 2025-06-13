idf_component_register(SRCS "signal.c"
                    INCLUDE_DIRS "include" "../DRV2605driver/include" "../pin_config/include"
                    REQUIRES driver freeRTOS pin_config DRV2605driver esp_timer gptimer) 
