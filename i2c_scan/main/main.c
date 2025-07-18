#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/task.h"
                                     //FG MAX          //DRV 0x5A          //LIS3D 0X19    //PMIC
#define I2C_MASTER_SCL_IO          13//9               //7                 //4             //13 
#define I2C_MASTER_SDA_IO          10//8               //6                 //3             //10
#define I2C_MASTER_NUM              I2C_NUM_0 
#define I2C_MASTER_FREQ_HZ          1000000
#define I2C_MASTER_TX_BUF_LEN       0
#define I2C_MASTER_RX_BUF_LEN       0

#include "driver/gpio.h"
#include "hal/gpio_hal.h"
#include "soc/gpio_reg.h"

// Deshabilita completamente los pines de los otros 3 buses I2C
void disable_unused_i2c_pins(void) {
    // Pines de los buses I2C que NO queremos usar:
    // FG MAX: SCL=9, SDA=8
    // DRV: SCL=7, SDA=6  
    // LIS3D: SCL=4, SDA=3
    int unused_pins[] = {3, 4, 6, 7, 8, 9};
    
    for (int i = 0; i < sizeof(unused_pins)/sizeof(unused_pins[0]); ++i) {
        int pin = unused_pins[i];
        
        // Resetear completamente el pin
        gpio_reset_pin(pin);
        
        // Configurar como entrada con pull-down para evitar floating
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << pin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,  // Pull-down para evitar estados flotantes
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        
        // Deshabilitar cualquier función especial del pin
        gpio_set_level(pin, 0);
        
        // Log para debug
        ESP_LOGI("GPIO_DISABLE", "Pin %d disabled and pulled down", pin);
    }
    
    // Asegurar que los drivers I2C de los otros buses estén desinstalados
    // Solo mantenemos I2C_NUM_0 para nuestro bus (pines 18, 10)
    i2c_driver_delete(I2C_NUM_1);  // Por si acaso estaba instalado
    
    ESP_LOGI("GPIO_DISABLE", "All unused I2C pins disabled. Only using SCL=18, SDA=10");
}

// Llama a esta función al inicio de app_main:
__attribute__((constructor))
static void setup_gpio_inputs(void) {
    disable_unused_i2c_pins();
}

// Configurar exclusivamente los pines del bus I2C que queremos usar
void configure_active_i2c_pins(void) {
    // Configurar específicamente los pines 18 (SCL) y 10 (SDA) para I2C
    ESP_LOGI("I2C_CONFIG", "Configuring active I2C pins: SCL=%d, SDA=%d", 
             I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO);
             
    // Resetear los pines antes de configurarlos
    gpio_reset_pin(I2C_MASTER_SCL_IO);
    gpio_reset_pin(I2C_MASTER_SDA_IO);
    
    // No necesitamos configuración manual adicional aquí,
    // el driver I2C se encargará de la configuración específica
}

static const char *SCANNER_TAG = "I2C_SCANNER";

static esp_err_t i2c_master_init_scanner(void) {
    // Asegurar que cualquier driver I2C previo esté limpio
    i2c_driver_delete(I2C_MASTER_NUM);
    
    i2c_config_t i2c_conf = {};
    i2c_conf.mode = I2C_MODE_MASTER;
    i2c_conf.sda_io_num = I2C_MASTER_SDA_IO;
    i2c_conf.scl_io_num = I2C_MASTER_SCL_IO;
    i2c_conf.sda_pullup_en = true;
    i2c_conf.scl_pullup_en = true;
    i2c_conf.master.clk_speed = 100000;  // 100kHz para mayor compatibilidad
    i2c_conf.clk_flags = 0;
    
    ESP_LOGI(SCANNER_TAG, "Configuring I2C on SCL=%d, SDA=%d", I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO);
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0));
    
    ESP_LOGI(SCANNER_TAG, "I2C driver installed successfully on pins SCL=%d, SDA=%d", 
             I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO);

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
    ESP_LOGI("MAIN", "=== Iniciando I2C Scanner ===");
    ESP_LOGI("MAIN", "Bus I2C activo: SCL=%d, SDA=%d", I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO);
    
    // 1. Deshabilitar pines de buses I2C no utilizados
    setup_gpio_inputs();
    
    // 2. Configurar pines del bus I2C activo
    configure_active_i2c_pins();
    
    // 3. Esperar un momento para estabilización
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // 4. Crear tarea de scanner I2C
    xTaskCreate(i2c_scanner_task, "i2c_scanner", 4096, NULL, 5, NULL);
    
    // 5. Mantener el programa corriendo
    while(1) {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}