
#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esp_log.h"
#include "driver/i2c.h"

#define I2C_SLAVE_ADDR 0x68
#define TIMEOOUT 1000
#define Delay_ms 1000

static const char *TAG = "I2C_SLAVE";

static esp_err_t set_i2c(void){
    i2c_config_t i2c_conf = {};

    i2c_conf.mode = I2C_MODE_MASTER;
    i2c_conf.sda_io_num = 1;
    i2c_conf.scl_io_num = 2;
    i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.master.clk_speed = 400000;
    i2c_conf.clk_flags = 0;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_conf));
    
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));


    return ESP_OK;
}

void app_main(void){
    ESP_ERROR_CHECK(set_i2c());
    uint8_t rx_data[12];
    uint8_t command = 0x00;
    while (1)
    {
        i2c_master_write_read_device(I2C_NUM_0, I2C_SLAVE_ADDR, &command, 1, rx_data, 12, pdMS_TO_TICKS(TIMEOOUT));
        ESP_LOG_BUFFER_HEX(TAG, rx_data, 12);
        vTaskDelay(pdMS_TO_TICKS(Delay_ms));
    }
    
}