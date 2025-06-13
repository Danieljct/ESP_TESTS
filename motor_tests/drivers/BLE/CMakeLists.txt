idf_component_register(SRCS "BLE.c"
                    INCLUDE_DIRS "include" "../pin_config/include" 
                    REQUIRES freertos nvs_flash esp_bt esp_gatt_common esp_gatts esp_gap esp32 bt pin_config
                    )
