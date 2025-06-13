#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/task.h"
                                     //FG MAX          //DRV 0x5A          //LIS3D 0X19    //PMIC
#define I2C_MASTER_SCL_IO          9//9               //7                 //4             //18 
#define I2C_MASTER_SDA_IO          8//8               //6                 //3             //10
#define I2C_MASTER_NUM              I2C_NUM_0 
#define I2C_MASTER_FREQ_HZ          1000000
#define I2C_MASTER_TX_BUF_LEN       0
#define I2C_MASTER_RX_BUF_LEN       0

#include "driver/gpio.h"

// Deshabilita totalmente los pines configurándolos como entradas sin pull-up ni pull-down
void configure_input_pins_with_pullup(void) {
    int pins[] = {3, 4, 6, 7, 10, 18};
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_DISABLE, // Deshabilita la función GPIO
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    for (int i = 0; i < sizeof(pins)/sizeof(pins[0]); ++i) {
        io_conf.pin_bit_mask = 1ULL << pins[i];
        gpio_config(&io_conf);
    }
}

// Llama a esta función al inicio de app_main:
__attribute__((constructor))
static void setup_gpio_inputs(void) {
    configure_input_pins_with_pullup();
}
static const char *SCANNER_TAG = "I2C_SCANNER";

static esp_err_t i2c_master_init_scanner(void) {
    i2c_config_t i2c_conf = {};

    i2c_conf.mode = I2C_MODE_MASTER;
    i2c_conf.sda_io_num = I2C_MASTER_SDA_IO;
    i2c_conf.scl_io_num = I2C_MASTER_SCL_IO;
    i2c_conf.sda_pullup_en = 1;
    i2c_conf.scl_pullup_en = 1;
    i2c_conf.master.clk_speed = 100000;
    i2c_conf.clk_flags = 0;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_conf));
    
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));


    return ESP_OK;
}

void i2c_scanner_task(void *pvParameters) {
    ESP_ERROR_CHECK(i2c_master_init_scanner());
    ESP_LOGI(SCANNER_TAG, "Starting I2C scan...");

    printf("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\r\n");
    for (int i = 0; i < 128; i += 16) {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j++) {
            int address = i + j;
            if (address == 0 || address == 127) { // Direcciones reservadas
                printf("   ");
                continue;
            }
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(cmd);
            esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 500 / portTICK_PERIOD_MS); // Pequeño timeout
            i2c_cmd_link_delete(cmd);
            if (ret == ESP_OK) {
                printf("%02x ", address);
            } else if (ret == ESP_ERR_TIMEOUT) {
                printf("TT "); // Timeout
            } else {
                printf("-- "); // NACK u otro error
            }
        }
        printf("\r\n");
    }
    ESP_LOGI(SCANNER_TAG, "I2C scan complete.");
    
    vTaskDelete(NULL); // Elimina la tarea después de escanear
}

// Para usarlo en app_main:
// xTaskCreate(i2c_scanner_task, "i2c_scanner", 4096, NULL, 5, NULL);
// Y luego comenta la inicialización y el uso del MAX17055 hasta que confirmes la conexión.
void app_main(void){
   setup_gpio_inputs();
   vTaskDelay(5000 / portTICK_PERIOD_MS); // Espera un segundo para que el sistema esté listo
     xTaskCreate(i2c_scanner_task, "i2c_scanner", 4096, NULL, 5, NULL);
      vTaskDelay(200000 / portTICK_PERIOD_MS); // Espera un segundo para que el sistema esté listo
}