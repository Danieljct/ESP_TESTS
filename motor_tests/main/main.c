#include <stdio.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "DRV2605driver.h" // Incluye tu driver DRV2605
#include "BLE.h"           // Incluye el módulo BLE que acabamos de adaptar
#include "signal.h"

static const char *TAG_MAIN = "MAIN";

// Declaración de la cola de comandos (definida en ble.c)
extern QueueHandle_t ble_command_queue;

void app_main(void) {
    esp_err_t ret;

    // 1. Inicializar NVS (Non-Volatile Storage) para BLE
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG_MAIN, "NVS inicializado.");

    // 2. Inicializar el driver DRV2605
    // Asegúrate de que mot_init() no dependa de BLE para su inicialización
    if (mot_init()) {
        ESP_LOGI(TAG_MAIN, "DRV2605 inicializado correctamente.");
    } else {
        ESP_LOGE(TAG_MAIN, "Fallo al inicializar DRV2605. Verifique conexiones I2C y pines.");
        // Considera si quieres detener la ejecución o intentar continuar
    }

    // 3. Inicializar el módulo BLE (tu servicio de comandos)
    ble_init();
    ESP_LOGI(TAG_MAIN, "BLE inicializado. Esperando conexión...");
SIGNAL_init(PWM_FREQ);
SIGNAL_select(SIGNAL_SIN23HZ);
SIGNAL_start();
    // Bucle principal para procesar comandos BLE
    ble_command_t received_command;

    while (1) {
        // Esperar por comandos de la cola (bloqueante hasta que haya algo)
        if (xQueueReceive(ble_command_queue, &received_command, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG_MAIN, "Comando recibido para procesamiento: %s", received_command.command);

            // Aquí procesas el comando recibido y actúas sobre el DRV2605
            // Luego, envías una respuesta de vuelta al cliente BLE.

            // Ejemplo de procesamiento de comando "READ_REG <registro>"
            if (strncmp(received_command.command, "READ_REG", 8) == 0) {
                uint8_t reg_addr;
                char response_str[BLE_MAX_DATA_LEN];

                if (sscanf(received_command.command + 9, "%hhu", &reg_addr) == 1) {
                    uint8_t value;
                    if (readRegister8(reg_addr, &value)) {
                        snprintf(response_str, sizeof(response_str), "READ_OK %02X:%02X", reg_addr, value);
                        ESP_LOGI(TAG_MAIN, "Respuesta a comando: %s", response_str);
                    } else {
                        snprintf(response_str, sizeof(response_str), "READ_ERROR %02X", reg_addr);
                        ESP_LOGE(TAG_MAIN, "Fallo lectura de registro 0x%02X", reg_addr);
                    }
                } else {
                    snprintf(response_str, sizeof(response_str), "READ_ERROR: Invalid format");
                    ESP_LOGE(TAG_MAIN, "Formato de comando READ_REG inválido.");
                }
                ble_send_response(response_str); // Enviar respuesta al cliente BLE
            }
            // Ejemplo de procesamiento de comando "WRITE_REG <registro> <valor>"
            else if (strncmp(received_command.command, "WRITE_REG", 9) == 0) {
                uint8_t reg_addr, value;
                char response_str[BLE_MAX_DATA_LEN];

                if (sscanf(received_command.command + 10, "%hhu %hhu", &reg_addr, &value) == 2) {
                    if (writeRegister8(reg_addr, value)) {
                        snprintf(response_str, sizeof(response_str), "WRITE_OK %02X:%02X", reg_addr, value);
                        ESP_LOGI(TAG_MAIN, "Respuesta a comando: %s", response_str);
                    } else {
                        snprintf(response_str, sizeof(response_str), "WRITE_ERROR %02X:%02X", reg_addr, value);
                        ESP_LOGE(TAG_MAIN, "Fallo escritura de registro 0x%02X con valor 0x%02X", reg_addr, value);
                    }
                } else {
                    snprintf(response_str, sizeof(response_str), "WRITE_ERROR: Invalid format");
                    ESP_LOGE(TAG_MAIN, "Formato de comando WRITE_REG inválido.");
                }
                ble_send_response(response_str);
            }
            // Puedes agregar más comandos aquí (ej. "SET_MODE <mode>", "CALIBRATE")
            else if (strcmp(received_command.command, "CALIBRATE") == 0) {
                char response_str[BLE_MAX_DATA_LEN];
                if (autoCalibration()) {
                    snprintf(response_str, sizeof(response_str), "CALIBRATION_OK");
                    ESP_LOGI(TAG_MAIN, "Calibración exitosa.");
                } else {
                    snprintf(response_str, sizeof(response_str), "CALIBRATION_FAIL");
                    ESP_LOGE(TAG_MAIN, "Fallo la calibración.");
                }
                ble_send_response(response_str);
            }
            else if (strcmp(received_command.command, "MOT_ACTIVE") == 0) {
                motorSetActiveMode();
                ble_send_response("MOT_ACTIVE_OK");
            }
            else if (strcmp(received_command.command, "MOT_OFF") == 0) {
                motorSetOffMode();
                ble_send_response("MOT_OFF_OK");
            }
            else if (strncmp(received_command.command, "SET_INPUT_MODE", 14) == 0) {
                uint8_t mode_val;
                char response_str[BLE_MAX_DATA_LEN];
                if (sscanf(received_command.command + 15, "%hhu", &mode_val) == 1) {
                    motorSetInputMode((motInMode)mode_val);
                    snprintf(response_str, sizeof(response_str), "SET_INPUT_MODE_OK %hhu", mode_val);
                } else {
                    snprintf(response_str, sizeof(response_str), "SET_INPUT_MODE_ERROR: Invalid format");
                }
                ble_send_response(response_str);
            }
            else {
                ESP_LOGW(TAG_MAIN, "Comando desconocido: %s", received_command.command);
                ble_send_response("UNKNOWN_COMMAND");
            }
        }
    }
}