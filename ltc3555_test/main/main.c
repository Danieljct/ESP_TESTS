#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/task.h"

// Configuración de pines
#define I2C_MASTER_SCL_IO          13
#define I2C_MASTER_SDA_IO          10
#define I2C_MASTER_NUM             I2C_NUM_0 
#define I2C_MASTER_FREQ_HZ         100000  // 100kHz según especificaciones del LTC3555

// LEDs de estado
#define LED_R 21
#define LED_G 18
#define LED_B 17

// LTC3555 configuración - SW2 variable, SW3 fijo en máximo (0.800V)
#define LTC3555_I2C_ADDR           0x09    // Dirección I2C del LTC3555
#define LTC3555_REG_A              0x00    // Registro A: A7-A4=SW2 (variable), A3-A0=SW3 (fijo en 1111=0.800V)
#define LTC3555_REG_B              0x01    // Registro B: B3=Enable SW2, B2=Enable SW3 (siempre 1)

static const char *TAG = "LTC3555_SW2_SW3";

// Estructura para configuraciones de voltaje
typedef struct {
    const char* name;
    uint8_t reg_a_value;  // A7-A4=SW2 (variable), A3-A0=SW3 (siempre 0xF para 0.800V)
    uint8_t reg_b_value;  // B3=SW2 enable, B2=SW3 enable (siempre 1)
    uint32_t led_color;   // LED indicador (RGB)
} ltc3555_config_t;

/*
 * LTC3555 I2C Protocol (según datasheet):
 * START + DEVICE_ADDR + WRITE + ACK + BYTE_A + ACK + BYTE_B + ACK + STOP
 * 
 * SW3 alimenta al ESP32 - SIEMPRE fijo en 0.800V (máximo)
 * SW2 es configurable para pruebas
 * 
 * BYTE A (Voltajes):
 * - Bits A7-A4 (SW2): Variable según configuración
 * - Bits A3-A0 (SW3): SIEMPRE 1111 (0.800V máximo)
 * 
 * BYTE B (Enables):
 * - Bit B3 = SW2 habilitado/deshabilitado
 * - Bit B2 = SW3 habilitado (SIEMPRE 1, forzado por software)
 * - Otros bits para límites de corriente, etc.
 * 
 * Tabla de voltajes (4 bits):
 * 0000 = 0.425V    0100 = 0.525V    1000 = 0.625V    1100 = 0.725V
 * 0001 = 0.450V    0101 = 0.550V    1001 = 0.650V    1101 = 0.750V
 * 0010 = 0.475V    0110 = 0.575V    1010 = 0.675V    1110 = 0.775V
 * 0011 = 0.500V    0111 = 0.600V    1011 = 0.700V    1111 = 0.800V
 */

// Configuraciones predefinidas de voltaje - SW2 variable, SW3 fijo en máximo (0.800V)
static const ltc3555_config_t voltage_configs[] = {
    // SW3 SIEMPRE en 0.800V (bits A3-A0 = 1111) para máxima alimentación del ESP32
    // SW3 SIEMPRE habilitado (bit B2 se fuerza a 1 en la función de escritura)
    // Solo variamos SW2 (bits A7-A4) y su habilitación (bit B3)
    
    // Configuración 1: SW2 = 0.425V ON, SW3 = 0.800V ON - LED ROJO
    {"SW2: 0.425V, SW3: 0.800V", 0x0F, 0x08, LED_R},  // A7-A4=0000 (SW2=0.425V), A3-A0=1111 (SW3=0.800V), B3=1 (SW2 ON), B2 se fuerza a 1
    
    // Configuración 2: SW2 = 0.500V ON, SW3 = 0.800V ON - LED VERDE
    {"SW2: 0.500V, SW3: 0.800V", 0x3F, 0x08, LED_G},  // A7-A4=0011 (SW2=0.500V), A3-A0=1111 (SW3=0.800V), B3=1 (SW2 ON), B2 se fuerza a 1
    
    // Configuración 3: SW2 = 0.600V ON, SW3 = 0.800V ON - LED AZUL
    {"SW2: 0.600V, SW3: 0.800V", 0x7F, 0x08, LED_B},  // A7-A4=0111 (SW2=0.600V), A3-A0=1111 (SW3=0.800V), B3=1 (SW2 ON), B2 se fuerza a 1
    
    // Configuración 4: SW2 = 0.725V ON, SW3 = 0.800V ON - LED ROJO+VERDE
    {"SW2: 0.725V, SW3: 0.800V", 0xCF, 0x08, LED_R | LED_G},  // A7-A4=1100 (SW2=0.725V), A3-A0=1111 (SW3=0.800V), B3=1 (SW2 ON), B2 se fuerza a 1
    
    // Configuración 5: SW2 = 0.800V ON, SW3 = 0.800V ON - LED ROJO+AZUL
    {"SW2: 0.800V, SW3: 0.800V", 0xFF, 0x08, LED_R | LED_B},  // A7-A4=1111 (SW2=0.800V), A3-A0=1111 (SW3=0.800V), B3=1 (SW2 ON), B2 se fuerza a 1
    
    // Configuración 6: SW2 OFF, SW3 = 0.800V ON - LED VERDE+AZUL
    {"SW2: OFF, SW3: 0.800V", 0x0F, 0x00, LED_G | LED_B},  // SW2 deshabilitado (B3=0), SW3 habilitado se garantiza (B2 se fuerza a 1)
};

static esp_err_t ltc3555_i2c_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando I2C: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error instalando driver I2C: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C inicializado - SCL: %d, SDA: %d, Freq: %d Hz", 
             I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, I2C_MASTER_FREQ_HZ);
    return ESP_OK;
}

static void led_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_R) | (1ULL << LED_G) | (1ULL << LED_B),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    // Apagar todos los LEDs al inicio
    gpio_set_level(LED_R, 0);
    gpio_set_level(LED_G, 0);
    gpio_set_level(LED_B, 0);
}

static void set_led_color(uint32_t color) {
    gpio_set_level(LED_R, (color & LED_R) ? 1 : 0);
    gpio_set_level(LED_G, (color & LED_G) ? 1 : 0);
    gpio_set_level(LED_B, (color & LED_B) ? 1 : 0);
}

static esp_err_t ltc3555_write_config(uint8_t reg_a_data, uint8_t reg_b_data) {
    // SEGURIDAD: Asegurar que B2 (Enable SW3) esté SIEMPRE en 1
    uint8_t safe_reg_b = reg_b_data | 0x04;  // Forzar bit B2 = 1 (SW3 enable)
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LTC3555_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_a_data, true);     // Escribir byte A (voltajes SW2 y SW3)
    i2c_master_write_byte(cmd, safe_reg_b, true);     // Escribir byte B (enables, SW3 protegido)
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error escribiendo configuración LTC3555: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "LTC3555 configurado: ByteA=0x%02X, ByteB=0x%02X (SW3 protegido)", reg_a_data, safe_reg_b);
    }
    
    return ret;
}

static esp_err_t ltc3555_apply_config(const ltc3555_config_t *config) {
    ESP_LOGI(TAG, "=== Aplicando configuración: %s ===", config->name);
    
    // Establecer color del LED
    set_led_color(config->led_color);
    
    ESP_LOGI(TAG, "Enviando configuración I2C:");
    ESP_LOGI(TAG, "  Byte A = 0x%02X (SW2+SW3 voltages)", config->reg_a_value);
    ESP_LOGI(TAG, "  Byte B = 0x%02X → 0x%02X (enables, SW3 forzado ON)", config->reg_b_value, config->reg_b_value | 0x04);
    
    esp_err_t ret = ltc3555_write_config(config->reg_a_value, config->reg_b_value);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ESP_LOGI(TAG, "✓ Configuración enviada. Mide SW2 con el tester.");
    ESP_LOGI(TAG, "✓ SW3 garantizado en ON para alimentar ESP32.");
    
    return ESP_OK;
}

static void ltc3555_test_task(void *pvParameters) {
    ESP_LOGI(TAG, "Iniciando prueba de configuraciones LTC3555 - Solo escritura");
    ESP_LOGI(TAG, "Mide el voltaje de SW2 con el tester durante cada configuración");
    
    const int num_configs = sizeof(voltage_configs) / sizeof(voltage_configs[0]);
    int current_config = 0;
    
    while (1) {
        ESP_LOGI(TAG, "\n========================================");
        ESP_LOGI(TAG, "Configuración %d/%d", current_config + 1, num_configs);
        ESP_LOGI(TAG, "========================================");
        
        esp_err_t ret = ltc3555_apply_config(&voltage_configs[current_config]);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✓ Comandos I2C enviados correctamente");
            ESP_LOGI(TAG, ">>> MIDE SW2 AHORA CON EL TESTER <<<");
            ESP_LOGI(TAG, "Esperando 15 segundos para medición...");
            
            // Countdown para dar tiempo de medición
            for (int i = 15; i > 0; i--) {
                ESP_LOGI(TAG, "Tiempo restante: %d segundos", i);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            
        } else {
            ESP_LOGE(TAG, "❌ Error enviando comandos I2C");
            // En caso de error, parpadear LED rojo
            for (int i = 0; i < 10; i++) {
                set_led_color(LED_R);
                vTaskDelay(200 / portTICK_PERIOD_MS);
                set_led_color(0);
                vTaskDelay(200 / portTICK_PERIOD_MS);
            }
        }
        
        // Pasar a la siguiente configuración
        current_config = (current_config + 1) % num_configs;
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "=== Iniciando Control LTC3555 ===");
    
    // Inicializar LEDs
    led_init();
    ESP_LOGI(TAG, "LEDs inicializados");
    
    // Parpadeo inicial de todos los LEDs
    for (int i = 0; i < 3; i++) {
        set_led_color(LED_R | LED_G | LED_B);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        set_led_color(0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
    // Inicializar I2C
    if (ltc3555_i2c_init() != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando I2C");
        // LED rojo fijo en caso de error
        set_led_color(LED_R);
        return;
    }
    
    // Crear tarea de prueba
    xTaskCreate(ltc3555_test_task, "ltc3555_test", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Sistema iniciado. Monitoreando configuraciones LTC3555...");
}