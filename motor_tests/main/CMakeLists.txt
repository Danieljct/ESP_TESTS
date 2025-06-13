idf_component_register(SRCS         "main.c" "../drivers/BLE/BLE.c" 
                                    "../drivers/DRV2605driver/DRV2605driver.c"
                                    "../drivers/signal/signal.c"

                    INCLUDE_DIRS    "../drivers/BLE/include" "../drivers/DRV2605driver/include" 
                                    "../drivers/pin_config/include"
                                    "../drivers/signal/include" )

list(APPEND EXTRA_COMPONENT_DIRS
    drivers
)