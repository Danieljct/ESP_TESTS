#ifndef BLE_H_
#define BLE_H_

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt.h"
#include "esp_gatt_common_api.h" // Para esp_ble_gatts_set_attr_value
#include <inttypes.h>            // Para PRIu32
#include "nvs_flash.h"           // Para nvs_flash_init, ESP_ERR_NVS_*

// Macros para extraer bytes (CONSTANTES DE PREPROCESADOR)
#define LO_BYTE(x) ((uint8_t)((x) & 0xFF))
#define HI_BYTE(x) ((uint8_t)(((x) >> 8) & 0xFF))

// UUIDs para el servicio y la característica de comandos/respuestas
#define GATTS_SERVICE_UUID_COMMANDS   0x00AA // UUID para tu servicio principal de comandos
#define GATTS_CHAR_UUID_COMMAND       0xAA01 // Característica para enviar comandos a la ESP32 (Write)
#define GATTS_CHAR_UUID_RESPONSE      0xAA02 // Característica para recibir respuestas de la ESP32 (Notify, Read)

#define BLE_MAX_COMMAND_LEN 250 // Longitud máxima del comando BLE
#define BLE_MAX_DATA_LEN    250 // Longitud máxima de los datos para la característica

// Estructura para los comandos que se pasarán a la cola
typedef struct {
    char command[BLE_MAX_COMMAND_LEN];
} ble_command_t;

// Estructura para almacenar los handles del perfil BLE de comandos
typedef struct {
    uint16_t gatts_if;
    uint16_t conn_id;
    bool is_connected;
    esp_bd_addr_t remote_bda;

    uint16_t commands_service_handle;
    uint16_t command_char_handle;
    uint16_t response_char_handle;
    uint16_t response_descr_handle; // CCCD para la característica de respuesta
    bool response_notify_enabled;
} ble_profile_data_t;


// Prototipos de funciones
void ble_init(void); // Ahora para el servicio de comandos
void ble_send_response(const char *response);

// Declaración externa de la cola de comandos
extern QueueHandle_t ble_command_queue;

#endif /* BLE_H_ */