#include <stdio.h>
#include <stdlib.h> // Necesario para malloc/free
#include <string.h> // Necesario para memcpy/memset
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "driver/gpio.h"
#include "esp_gatt_defs.h" // Para UUIDs estándar como ESP_GATT_UUID_BATTERY_SERVICE_SVC
#include "freertos/timers.h"


// Incluye tu header si tienes uno para este módulo (ej: BLE.h)
#include "BLE.h" // Si defines vibebrain_characteristics_t aquí


#define GATTS_TAG "VIBE_GATTS" // Tag para los logs

/// Declaración estática del manejador de perfil
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
// Declaración estática de la tarea de simulación
static void battery_simulation_task(void *pvParameters);
// Declaración de la función de actualización/notificación de batería
void ble_bas_update_battery_level(void);


// --- UUIDs y Handles BATERÍA (Estándar) ---
#define GATTS_SERVICE_UUID_BATTERY     ESP_GATT_UUID_BATTERY_SERVICE_SVC // 0x180F
#define GATTS_CHAR_UUID_BATTERY_LEVEL  ESP_GATT_UUID_BATTERY_LEVEL     // 0x2A19 (Notify, Read)
// #define GATTS_NUM_HANDLE_BATTERY       4 // Ya no se usa individualmente

// --- UUIDs y Handles VIBEBRAIN ---
#define GATTS_SERVICE_UUID_VIBEBRAIN   0x00AA // UUID para tu servicio principal
#define GATTS_CHAR_UUID_PERIOD         0xAA01 // Característica para Periodo
#define GATTS_CHAR_UUID_DUTY_CYCLE     0xAA02 // Duty Cycle (sin Notify)
#define GATTS_CHAR_UUID_PLAY_STOP      0xAA03 // Play/Stop
#define GATTS_CHAR_UUID_REPETITIONS    0xAA04 // Repetitions (CON Notify)
// #define GATTS_NUM_HANDLE_VIBEBRAIN     12 // Ya no se usa individualmente

// --- TOTAL HANDLES APROXIMADO PARA LA APP ---
// VibeBrain: 1(Svc)+4*2(Chars)+1(CCCD Reps)=10
// Battery:   1(Svc)+1*2(Char)+1(CCCD Batt)=4
// Total: 14 (Usar 20 como margen)
#define GATTS_TOTAL_HANDLES_APP0       20

// --- Variables Globales ---
// Estructura para valores de características VIBEBRAIN
vibebrain_characteristics_t vibebrain_chars = {
    .period = PULSE_PERIOD,
    .repetitions = MAX_REPETITIONS,
    .play_stop = false,
    .duty_cycle = PULSE_DUTY
};
static volatile bool shutdown_requested_via_ble = false;

// Variable para Nivel de Batería simulado
static uint8_t battery_level = 100; // Iniciar al 100%

// --- Estructuras de Atributos GATT ---
// (Las de VibeBrain: gatts_char_period_val, gatts_char_duty_cycle_val, etc.)
static esp_attr_value_t gatts_char_period_val = {
    .attr_max_len = sizeof(vibebrain_chars.period),
    .attr_len     = sizeof(vibebrain_chars.period),
    .attr_value   = (uint8_t*)&(vibebrain_chars.period),
};
static esp_attr_value_t gatts_char_duty_cycle_val = {
    .attr_max_len = sizeof(vibebrain_chars.duty_cycle),
    .attr_len     = sizeof(vibebrain_chars.duty_cycle),
    .attr_value   = (uint8_t*)&(vibebrain_chars.duty_cycle),
};
static esp_attr_value_t gatts_char_play_stop_val = {
    .attr_max_len = sizeof(vibebrain_chars.play_stop),
    .attr_len     = sizeof(vibebrain_chars.play_stop),
    .attr_value   = (uint8_t*)&(vibebrain_chars.play_stop),
};
static esp_attr_value_t gatts_char_repetitions_val = {
    .attr_max_len = sizeof(vibebrain_chars.repetitions),
    .attr_len     = sizeof(vibebrain_chars.repetitions),
    .attr_value   = (uint8_t*)&(vibebrain_chars.repetitions),
};

// Atributo para Nivel de Batería
static esp_attr_value_t gatts_char_battery_level_val = {
    .attr_max_len = sizeof(battery_level),
    .attr_len     = sizeof(battery_level),
    .attr_value   = (uint8_t*)&battery_level,
};


// --- Estructura de Perfil (CORREGIDA con handles separados) ---
#define PROFILE_NUM 1
#define PROFILE_APP_ID 0

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    bool is_connected;
    esp_bd_addr_t remote_bda;

    // Handles Servicio VibeBrain
    uint16_t vibebrain_service_handle; // <-- Handle específico VibeBrain
    uint16_t period_char_handle;
    uint16_t duty_cycle_char_handle;
    uint16_t play_stop_char_handle;
    uint16_t repetitions_char_handle;
    uint16_t repetitions_descr_handle;
    bool repetitions_notify_enabled;

    // Handles Servicio Batería
    uint16_t battery_service_handle;   // <-- Handle específico Batería
    uint16_t battery_level_char_handle;
    uint16_t battery_level_descr_handle;
    bool battery_notify_enabled;
};

// --- Instancia del Perfil (Inicializar handles a 0) ---
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_ID] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,
        .is_connected = false,
        .repetitions_notify_enabled = false,
        .battery_notify_enabled = false,
        .vibebrain_service_handle = 0, // <-- Inicializar
        .battery_service_handle = 0,   // <-- Inicializar
        // ... (otros handles también se inicializan a 0 implícitamente) ...
    }
};

// --- Configuración Advertising (sin cambios) ---
static char test_device_name[ESP_BLE_ADV_NAME_LEN_MAX] = "VIBEBRAIN";
static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)
static uint8_t adv_service_uuid128[16] = { /* LSB <--------> MSB */ 0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, (uint8_t)(GATTS_SERVICE_UUID_VIBEBRAIN & 0xFF), (uint8_t)((GATTS_SERVICE_UUID_VIBEBRAIN >> 8) & 0xFF), 0x00, 0x00, };
static esp_ble_adv_data_t adv_data = { .set_scan_rsp = false, .include_name = true, .include_txpower = false, .min_interval = 0x0006, .max_interval = 0x0010, .appearance = 0x00, .manufacturer_len = 0, .p_manufacturer_data =  NULL, .service_data_len = 0, .p_service_data = NULL, .service_uuid_len = sizeof(adv_service_uuid128), .p_service_uuid = adv_service_uuid128, .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT), };
static esp_ble_adv_data_t scan_rsp_data = { .set_scan_rsp = true, .include_name = true, .include_txpower = true, .appearance = 0x00, .manufacturer_len = 0, .p_manufacturer_data =  NULL, .service_data_len = 0, .p_service_data = NULL, .service_uuid_len = 0, .p_service_uuid = NULL, .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT), };
static esp_ble_adv_params_t adv_params = { .adv_int_min = 0x20, .adv_int_max = 0x40, .adv_type = ADV_TYPE_IND, .own_addr_type = BLE_ADDR_TYPE_PUBLIC, .channel_map = ADV_CHNL_ALL, .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY, };

// --- Manejo escrituras preparadas (sin cambios) ---
#define PREPARE_BUF_MAX_SIZE 1024
typedef struct { uint8_t *prepare_buf; int prepare_len; } prepare_type_env_t;
static prepare_type_env_t prepare_write_env;
// void example_write_event_env(...) { /* ... */ } // Asumiendo que no se usan escrituras preparadas largas
// void example_exec_write_event_env(...) { /* ... */ }


// --- Manejador de eventos GAP (sin cambios) ---
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
     switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT: adv_config_done &= (~adv_config_flag); if (adv_config_done == 0) esp_ble_gap_start_advertising(&adv_params); break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT: adv_config_done &= (~scan_rsp_config_flag); if (adv_config_done == 0) esp_ble_gap_start_advertising(&adv_params); break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT: if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) ESP_LOGE(GATTS_TAG, "Adv start failed"); else ESP_LOGI(GATTS_TAG, "Adv start success"); break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT: if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) ESP_LOGE(GATTS_TAG, "Adv stop failed"); else ESP_LOGI(GATTS_TAG, "Adv stop success"); break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT: ESP_LOGI(GATTS_TAG, "Update conn params status=%d, min_int=%d, max_int=%d, conn_int=%d, latency=%d, timeout=%d", param->update_conn_params.status, param->update_conn_params.min_int, param->update_conn_params.max_int, param->update_conn_params.conn_int, param->update_conn_params.latency, param->update_conn_params.timeout); break;
        default: break;
     }
}

// --- Manejador de eventos GATTS (CORREGIDO) ---
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d, gatts_if %d", param->reg.status, param->reg.app_id, gatts_if);
            if (param->reg.status == ESP_GATT_OK) {
                gl_profile_tab[PROFILE_APP_ID].gatts_if = gatts_if;
            } else {
                ESP_LOGE(GATTS_TAG, "Reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
                return;
            }

            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(test_device_name);
            if (set_dev_name_ret) ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret) ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
            adv_config_done |= adv_config_flag;
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret) ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
            adv_config_done |= scan_rsp_config_flag;

            // Crear Servicio VibeBrain (0xAA)
            esp_gatt_srvc_id_t vibebrain_service_id;
            vibebrain_service_id.is_primary = true;
            vibebrain_service_id.id.inst_id = 0x00;
            vibebrain_service_id.id.uuid.len = ESP_UUID_LEN_16;
            vibebrain_service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_VIBEBRAIN;
            esp_ble_gatts_create_service(gatts_if, &vibebrain_service_id, GATTS_TOTAL_HANDLES_APP0);

            // La creación del servicio de batería se inicia DESPUÉS de que se complete la del primero (en CREATE_EVT)
            // para evitar problemas de timing y asegurar que se manejan secuencialmente.

            break;

        case ESP_GATTS_CREATE_EVT: { // Se disparará DOS VECES
            ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d, service_handle %d, service_uuid: 0x%04X",
                     param->create.status, param->create.service_handle, param->create.service_id.id.uuid.uuid.uuid16);

            if (param->create.status == ESP_GATT_OK) {
                uint16_t created_service_handle = param->create.service_handle;
                uint16_t created_service_uuid = param->create.service_id.id.uuid.uuid.uuid16;

                if (created_service_uuid == GATTS_SERVICE_UUID_VIBEBRAIN) {
                    ESP_LOGI(GATTS_TAG, "Servicio VibeBrain Creado (Handle: %d)", created_service_handle);
                    // --- GUARDAR HANDLE ESPECÍFICO ---
                    gl_profile_tab[PROFILE_APP_ID].vibebrain_service_handle = created_service_handle;
                    esp_ble_gatts_start_service(created_service_handle);

                    // Añadir características VibeBrain
                    esp_ble_gatts_add_char(created_service_handle, &(esp_bt_uuid_t){.len=ESP_UUID_LEN_16, .uuid.uuid16=GATTS_CHAR_UUID_PERIOD},      ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_WRITE, &gatts_char_period_val, NULL);
                    esp_ble_gatts_add_char(created_service_handle, &(esp_bt_uuid_t){.len=ESP_UUID_LEN_16, .uuid.uuid16=GATTS_CHAR_UUID_DUTY_CYCLE}, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_WRITE, &gatts_char_duty_cycle_val, NULL);
                    esp_ble_gatts_add_char(created_service_handle, &(esp_bt_uuid_t){.len=ESP_UUID_LEN_16, .uuid.uuid16=GATTS_CHAR_UUID_PLAY_STOP},  ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_WRITE, &gatts_char_play_stop_val, NULL);
                    esp_ble_gatts_add_char(created_service_handle, &(esp_bt_uuid_t){.len=ESP_UUID_LEN_16, .uuid.uuid16=GATTS_CHAR_UUID_REPETITIONS},ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_WRITE|ESP_GATT_CHAR_PROP_BIT_NOTIFY, &gatts_char_repetitions_val, NULL);

                    // --- Ahora que el primer servicio se creó, crear el segundo ---
                    ESP_LOGI(GATTS_TAG, "Iniciando creación de Servicio Batería...");
                    esp_gatt_srvc_id_t battery_service_id;
                    battery_service_id.is_primary = true;
                    battery_service_id.id.inst_id = 0x00;
                    battery_service_id.id.uuid.len = ESP_UUID_LEN_16;
                    battery_service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_BATTERY;
                    esp_ble_gatts_create_service(gatts_if, &battery_service_id, GATTS_TOTAL_HANDLES_APP0);


                } else if (created_service_uuid == GATTS_SERVICE_UUID_BATTERY) {
                    ESP_LOGI(GATTS_TAG, "Servicio Batería Creado (Handle: %d)", created_service_handle);
                     // --- GUARDAR HANDLE ESPECÍFICO ---
                    gl_profile_tab[PROFILE_APP_ID].battery_service_handle = created_service_handle;
                    esp_ble_gatts_start_service(created_service_handle);

                    // Añadir característica Nivel de Batería
                    esp_ble_gatts_add_char(created_service_handle, &(esp_bt_uuid_t){.len=ESP_UUID_LEN_16, .uuid.uuid16=GATTS_CHAR_UUID_BATTERY_LEVEL}, ESP_GATT_PERM_READ, ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY, &gatts_char_battery_level_val, NULL);
                }
            } else {
                ESP_LOGE(GATTS_TAG, "Create service failed, status %d", param->create.status);
            }
            break;
        } // Fin CREATE_EVT

        case ESP_GATTS_ADD_CHAR_EVT: {
            uint16_t attr_handle = param->add_char.attr_handle;
            uint16_t service_handle = param->add_char.service_handle; // Handle del servicio al que pertenece
            uint16_t char_uuid = param->add_char.char_uuid.uuid.uuid16;

            ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d, attr_handle %d, service_handle %d, char_uuid: 0x%04X",
                     param->add_char.status, attr_handle, service_handle, char_uuid);

             if (param->add_char.status != ESP_GATT_OK) {
                 ESP_LOGE(GATTS_TAG, "Failed to add characteristic, UUID=0x%04X, status=0x%X", char_uuid, param->add_char.status);
                 break; // Salir si falla la adición
             }

            bool needs_cccd = false;
            // --- Comprobar a qué servicio pertenece usando el handle CORRECTO ---
            if (service_handle == gl_profile_tab[PROFILE_APP_ID].vibebrain_service_handle) {
                ESP_LOGD(GATTS_TAG, "Char added to VibeBrain Service");
                if (char_uuid == GATTS_CHAR_UUID_PERIOD) { gl_profile_tab[PROFILE_APP_ID].period_char_handle = attr_handle; }
                else if (char_uuid == GATTS_CHAR_UUID_DUTY_CYCLE) { gl_profile_tab[PROFILE_APP_ID].duty_cycle_char_handle = attr_handle; }
                else if (char_uuid == GATTS_CHAR_UUID_PLAY_STOP) { gl_profile_tab[PROFILE_APP_ID].play_stop_char_handle = attr_handle; }
                else if (char_uuid == GATTS_CHAR_UUID_REPETITIONS) {
                    gl_profile_tab[PROFILE_APP_ID].repetitions_char_handle = attr_handle;
                    needs_cccd = true; // Repetitions necesita CCCD
                }
            } else if (service_handle == gl_profile_tab[PROFILE_APP_ID].battery_service_handle) {
                ESP_LOGD(GATTS_TAG, "Char added to Battery Service");
                if (char_uuid == GATTS_CHAR_UUID_BATTERY_LEVEL) {
                    gl_profile_tab[PROFILE_APP_ID].battery_level_char_handle = attr_handle;
                    needs_cccd = true; // Battery Level necesita CCCD
                }
            } else {
                 ESP_LOGW(GATTS_TAG, "Characteristic added to unknown service handle: %d", service_handle);
            }

            // Añadir CCCD si la característica lo necesita
            if (needs_cccd) {
                ESP_LOGI(GATTS_TAG, "Adding CCCD for char 0x%04X in service handle %d", char_uuid, service_handle);
                esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(
                                                service_handle, // Usar el handle del servicio correcto
                                                &(esp_bt_uuid_t){.len=ESP_UUID_LEN_16, .uuid.uuid16=ESP_GATT_UUID_CHAR_CLIENT_CONFIG},
                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                NULL, NULL); // Usar valor por defecto gestionado por stack
                if (add_descr_ret) {
                    ESP_LOGE(GATTS_TAG, "add char descr failed, error code = 0x%x", add_descr_ret);
                }
            }
            break;
        } // Fin ADD_CHAR_EVT

        case ESP_GATTS_ADD_CHAR_DESCR_EVT: {
            uint16_t service_handle = param->add_char_descr.service_handle;
            uint16_t descr_handle = param->add_char_descr.attr_handle;
            uint16_t descr_uuid = param->add_char_descr.descr_uuid.uuid.uuid16;

            ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d, descr_uuid: 0x%04X",
                     param->add_char_descr.status, descr_handle, service_handle, descr_uuid);

             if (param->add_char_descr.status != ESP_GATT_OK) {
                 ESP_LOGE(GATTS_TAG, "Failed to add descriptor, UUID=0x%04X, status=0x%X", descr_uuid, param->add_char_descr.status);
                 break;
             }

            // Guardar handle del CCCD en el lugar correcto
            if (descr_uuid == ESP_GATT_UUID_CHAR_CLIENT_CONFIG) {
                // --- Comprobar a qué servicio pertenece usando el handle CORRECTO ---
                if (service_handle == gl_profile_tab[PROFILE_APP_ID].vibebrain_service_handle) {
                    // Asumimos que es el CCCD de Repetitions porque es el único notificable en este servicio
                    gl_profile_tab[PROFILE_APP_ID].repetitions_descr_handle = descr_handle;
                    ESP_LOGI(GATTS_TAG, "Repetitions (0xAA04) CCCD handle stored: %d", descr_handle);
                } else if (service_handle == gl_profile_tab[PROFILE_APP_ID].battery_service_handle) {
                    // Asumimos que es el CCCD de Battery Level
                    gl_profile_tab[PROFILE_APP_ID].battery_level_descr_handle = descr_handle;
                    ESP_LOGI(GATTS_TAG, "Battery Level (0x2A19) CCCD handle stored: %d", descr_handle);
                } else {
                    ESP_LOGW(GATTS_TAG, "CCCD added to unknown service handle: %d", service_handle);
                }
            }
            break;
        } // Fin ADD_CHAR_DESCR_EVT

        case ESP_GATTS_READ_EVT: {
             ESP_LOGD(GATTS_TAG, "GATT_READ_EVT, handle %d", param->read.handle);
             esp_gatt_rsp_t rsp;
             memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
             rsp.attr_value.handle = param->read.handle;
             esp_gatt_status_t status = ESP_GATT_OK;

             // Comprobar handles específicos
             if (param->read.handle == gl_profile_tab[PROFILE_APP_ID].period_char_handle) {
                rsp.attr_value.len = sizeof(vibebrain_chars.period);
                memcpy(rsp.attr_value.value, &(vibebrain_chars.period), rsp.attr_value.len);
             } else if (param->read.handle == gl_profile_tab[PROFILE_APP_ID].duty_cycle_char_handle) {
                rsp.attr_value.len = sizeof(vibebrain_chars.duty_cycle);
                memcpy(rsp.attr_value.value, &(vibebrain_chars.duty_cycle), rsp.attr_value.len);
             } else if (param->read.handle == gl_profile_tab[PROFILE_APP_ID].play_stop_char_handle) {
                rsp.attr_value.len = sizeof(vibebrain_chars.play_stop);
                memcpy(rsp.attr_value.value, &(vibebrain_chars.play_stop), rsp.attr_value.len);
             } else if (param->read.handle == gl_profile_tab[PROFILE_APP_ID].repetitions_char_handle) {
                rsp.attr_value.len = sizeof(vibebrain_chars.repetitions);
                memcpy(rsp.attr_value.value, &(vibebrain_chars.repetitions), rsp.attr_value.len);
             } else if (param->read.handle == gl_profile_tab[PROFILE_APP_ID].battery_level_char_handle) {
                rsp.attr_value.len = sizeof(battery_level);
                memcpy(rsp.attr_value.value, &battery_level, rsp.attr_value.len);
             } else if (param->read.handle == gl_profile_tab[PROFILE_APP_ID].repetitions_descr_handle ||
                        param->read.handle == gl_profile_tab[PROFILE_APP_ID].battery_level_descr_handle) {
                 // Lectura de CCCD, dejar que el stack responda automáticamente (ESP_GATT_AUTO_RSP)
                 ESP_LOGI(GATTS_TAG, "Read request for CCCD handle %d", param->read.handle);
                 // No necesitamos enviar respuesta manual si se usa AUTO_RSP
                 // Sin embargo, para que la función termine limpiamente sin enviar doble respuesta:
                  rsp.attr_value.len = 0; // No enviar valor manualmente
                  status = ESP_GATT_OK;   // Indicar OK, pero AUTO_RSP se encargará del valor.
                  // NO LLAMAR a send_response manualmente para CCCD con AUTO_RSP
                  // Salir del case aquí para evitar el send_response de abajo? No, dejar que envíe OK sin datos.
             } else {
                  ESP_LOGW(GATTS_TAG, "Read request for unknown handle: %d", param->read.handle);
                  status = ESP_GATT_READ_NOT_PERMIT; // O ESP_GATT_INVALID_HANDLE
                  rsp.attr_value.len = 0;
             }
             // Enviar respuesta para lecturas de características (no CCCD con AUTO_RSP)
             // Nota: Si el CCCD se añadió SIN AUTO_RSP, sí habría que enviar rsp aquí.
             // Como lo añadimos con NULL, NULL, asumimos AUTO_RSP.
              ESP_LOGD(GATTS_TAG, "Sending read response for handle %d, status %d", param->read.handle, status);
              esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, status, &rsp);
             break;
        } // Fin READ_EVT


        case ESP_GATTS_WRITE_EVT: {
             ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, handle %d, len %d", param->write.handle, param->write.len);
             esp_gatt_status_t status = ESP_GATT_OK;
             bool Vibe_char = false; //flag para comprobar si se escribió en las características de vibe

             // 1. Comprobar CCCDs
             if (!param->write.is_prep) {
                 if (param->write.handle == gl_profile_tab[PROFILE_APP_ID].repetitions_descr_handle) {
                    if (param->write.len == 2) {
                         uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                         gl_profile_tab[PROFILE_APP_ID].repetitions_notify_enabled = (descr_value == 0x0001);
                         ESP_LOGI(GATTS_TAG, "Repetitions notify %s", gl_profile_tab[PROFILE_APP_ID].repetitions_notify_enabled ? "enabled" : "disabled");
                    } else { status = ESP_GATT_INVALID_ATTR_LEN; }
                 } else if (param->write.handle == gl_profile_tab[PROFILE_APP_ID].battery_level_descr_handle) {
                     if (param->write.len == 2) {
                         uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                         gl_profile_tab[PROFILE_APP_ID].battery_notify_enabled = (descr_value == 0x0001);
                         ESP_LOGI(GATTS_TAG, "Battery notify %s", gl_profile_tab[PROFILE_APP_ID].battery_notify_enabled ? "enabled" : "disabled");
                         if (gl_profile_tab[PROFILE_APP_ID].battery_notify_enabled) {
                             // Enviar valor actual inmediatamente al activar notificaciones
                             ble_bas_update_battery_level();
                         }
                     } else { status = ESP_GATT_INVALID_ATTR_LEN; }
                 }
                 // 2. Comprobar características VibeBrain
                 else if (param->write.handle == gl_profile_tab[PROFILE_APP_ID].period_char_handle) {
                     if (param->write.len == sizeof(vibebrain_chars.period)) {
                         memcpy(&(vibebrain_chars.period), param->write.value, param->write.len);
                         ESP_LOGI(GATTS_TAG, "Period updated: %"PRIu32, vibebrain_chars.period);
                         Vibe_char = true;
                     } else { status = ESP_GATT_INVALID_ATTR_LEN; }
                 } else if (param->write.handle == gl_profile_tab[PROFILE_APP_ID].duty_cycle_char_handle) {
                     if (param->write.len == sizeof(vibebrain_chars.duty_cycle)) {
                         memcpy(&(vibebrain_chars.duty_cycle), param->write.value, param->write.len);
                         ESP_LOGI(GATTS_TAG, "Duty updated: %u", vibebrain_chars.duty_cycle);
                          Vibe_char = true;
                     } else { status = ESP_GATT_INVALID_ATTR_LEN; }
                 } else if (param->write.handle == gl_profile_tab[PROFILE_APP_ID].play_stop_char_handle) {
                      if (param->write.len == sizeof(vibebrain_chars.play_stop)) {
                         memcpy(&(vibebrain_chars.play_stop), param->write.value, param->write.len);
                         ESP_LOGI(GATTS_TAG, "Play/Stop updated: %s", vibebrain_chars.play_stop ? "STOP" : "PLAY");
                         if(vibebrain_chars.play_stop) shutdown_requested_via_ble = true;
                      } else { status = ESP_GATT_INVALID_ATTR_LEN; }
                 } else if (param->write.handle == gl_profile_tab[PROFILE_APP_ID].repetitions_char_handle) {
                     if (param->write.len == sizeof(vibebrain_chars.repetitions)) {
                         memcpy(&(vibebrain_chars.repetitions), param->write.value, param->write.len);
                         ESP_LOGI(GATTS_TAG, "Repetitions updated: %"PRIu32, vibebrain_chars.repetitions);
                         // Considerar si notificar este cambio también
                         // ble_vibebrain_update_repetitions();
                      } else { status = ESP_GATT_INVALID_ATTR_LEN; }
                 }
                 else {
                     ESP_LOGW(GATTS_TAG, "Write EVT for unknown handle: %d", param->write.handle);
                     status = ESP_GATT_WRITE_NOT_PERMIT; // Opcional: indicar error si el handle no coincide con nada
                 }

                // Llamar a SIGNAL_setPulseTiming si cambió Period o Duty
                if(Vibe_char){SIGNAL_setPulseTiming(vibebrain_chars.period, vibebrain_chars.duty_cycle);}

             }
             // 3. Manejar escrituras preparadas (si se usan)
             else if (param->write.is_prep) {
                  // ... lógica de prepare write ...
                  // (Enviar respuesta de preparación)
                   status = ESP_GATT_INVALID_ATTR_LEN; // Indicar no soportado si no se implementa
             }

             // 4. Enviar respuesta final si es necesaria
             if (param->write.need_rsp){
                 ESP_LOGI(GATTS_TAG, "Enviando write response para handle %d, status %d", param->write.handle, status);
                 esp_err_t rsp_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
                 if (rsp_err != ESP_OK) { ESP_LOGE(GATTS_TAG, "Error enviando write response: 0x%x", rsp_err); }
             }
             break;
        } // Fin WRITE_EVT


        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT, conn_id %d, reason 0x%x", param->disconnect.conn_id, param->disconnect.reason);
            gl_profile_tab[PROFILE_APP_ID].is_connected = false;
            gl_profile_tab[PROFILE_APP_ID].repetitions_notify_enabled = false;
            gl_profile_tab[PROFILE_APP_ID].battery_notify_enabled = false; // Resetear flag batería
            shutdown_requested_via_ble = false;
            esp_ble_gap_start_advertising(&adv_params);
            break;

        // ... (Otros cases como CONNECT, MTU, etc., sin cambios relevantes) ...
         case ESP_GATTS_CONNECT_EVT: {
             ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote "ESP_BD_ADDR_STR"",
                      param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
             gl_profile_tab[PROFILE_APP_ID].conn_id = param->connect.conn_id;
             gl_profile_tab[PROFILE_APP_ID].is_connected = true;
             memcpy(gl_profile_tab[PROFILE_APP_ID].remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
             esp_ble_conn_update_params_t conn_params = {0};
             memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
             conn_params.latency = 0; conn_params.max_int = 0x20; conn_params.min_int = 0x10; conn_params.timeout = 400;
             esp_ble_gap_update_conn_params(&conn_params);
             break;
         }
         case ESP_GATTS_MTU_EVT: ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu); break;
         case ESP_GATTS_START_EVT: ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle); break;
         case ESP_GATTS_CONF_EVT: ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status %d, handle %d", param->conf.status, param->conf.handle); break;
         // ... otros cases ...
        default:
            ESP_LOGD(GATTS_TAG, "Unhandled GATTS Event: %d", event);
            break;
    } // Fin switch(event)
} // Fin gatts_profile_event_handler


// --- Manejador GATTS Principal ---
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) { gl_profile_tab[param->reg.app_id].gatts_if = gatts_if; }
        else { ESP_LOGE(GATTS_TAG, "Reg app failed"); return; }
    }
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) { gl_profile_tab[idx].gatts_cb(event, gatts_if, param); }
            }
        }
    } while (0);
}


// --- Funciones de Actualización y Notificación ---

/**
 * @brief Actualiza repeticiones y notifica si está habilitado.
 */
void ble_vibebrain_update_repetitions(void) {
     if (!gl_profile_tab[PROFILE_APP_ID].is_connected) return; // No hacer nada si no está conectado

     // 1. Actualizar valor GATT (Opcional si ya apunta a la variable global, pero bueno por si acaso)
     // esp_ble_gatts_set_attr_value(gl_profile_tab[PROFILE_APP_ID].repetitions_char_handle, sizeof(vibebrain_chars.repetitions), (uint8_t*)&(vibebrain_chars.repetitions));

     // 2. Notificar si está habilitado
     if (gl_profile_tab[PROFILE_APP_ID].repetitions_notify_enabled) {
         ESP_LOGD(GATTS_TAG, "Enviando notificación Repeticiones: %"PRIu32, vibebrain_chars.repetitions);
         esp_err_t err = esp_ble_gatts_send_indicate(
                            gl_profile_tab[PROFILE_APP_ID].gatts_if,
                            gl_profile_tab[PROFILE_APP_ID].conn_id,
                            gl_profile_tab[PROFILE_APP_ID].repetitions_char_handle,
                            sizeof(vibebrain_chars.repetitions),
                            (uint8_t*)&(vibebrain_chars.repetitions),
                            false); // Notificación
         if (err != ESP_OK) { ESP_LOGE(GATTS_TAG, "Error enviando notificación Repeticiones: 0x%x", err); }
     }
}

/**
 * @brief Actualiza batería y notifica si está habilitado.
 */
void ble_bas_update_battery_level(void) {
     if (!gl_profile_tab[PROFILE_APP_ID].is_connected) return;

     // 1. Actualizar valor GATT
     esp_err_t set_attr_ret = esp_ble_gatts_set_attr_value(
                                 gl_profile_tab[PROFILE_APP_ID].battery_level_char_handle,
                                 sizeof(battery_level),
                                 (const uint8_t *)&battery_level);
     if (set_attr_ret != ESP_OK) { ESP_LOGE(GATTS_TAG, "Error seteando valor attr batería: 0x%x", set_attr_ret); }

     // 2. Notificar si está habilitado
     if (gl_profile_tab[PROFILE_APP_ID].battery_notify_enabled) {
         ESP_LOGD(GATTS_TAG, "Enviando notificación Batería: %d%%", battery_level);
         esp_err_t err = esp_ble_gatts_send_indicate(
                            gl_profile_tab[PROFILE_APP_ID].gatts_if,
                            gl_profile_tab[PROFILE_APP_ID].conn_id,
                            gl_profile_tab[PROFILE_APP_ID].battery_level_char_handle,
                            sizeof(battery_level),
                            (uint8_t*)&battery_level,
                            false); // Notificación
         if (err != ESP_OK) { ESP_LOGE(GATTS_TAG, "Error enviando notificación Batería: 0x%x", err); }
     }
}

// --- Tarea Simulación Batería ---
#define BATTERY_UPDATE_INTERVAL_MS 1000

static void battery_simulation_task(void *pvParameters) {
    ESP_LOGI(GATTS_TAG, "Tarea simulación batería iniciada.");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(BATTERY_UPDATE_INTERVAL_MS));
        /*
        if (battery_level > 0) {
            battery_level--;
        } else {
            battery_level = 100; // Reset a 100% cuando llega a 0
        }
        */
        battery_level = (uint8_t)max17055_get_soc();
        ESP_LOGD(GATTS_TAG, "Nivel batería simulado: %d %%", battery_level);
        // Actualizar GATT y notificar (solo si está conectado y suscrito)
        ble_bas_update_battery_level();
    }
}


// --- Función Principal de Inicialización BLE ---
esp_err_t ble_vibebrain_init(void){
    esp_err_t ret;

    // Inicializar NVS (Necesario para BT)
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    // Inicializar Controlador BT
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) { ESP_LOGE(GATTS_TAG, "%s init controller failed: %s", __func__, esp_err_to_name(ret)); return ret; }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) { ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret)); return ret; }

    // Inicializar Bluedroid Stack
    ret = esp_bluedroid_init();
    if (ret) { ESP_LOGE(GATTS_TAG, "%s init bluedroid failed: %s", __func__, esp_err_to_name(ret)); return ret; }
    ret = esp_bluedroid_enable();
    if (ret) { ESP_LOGE(GATTS_TAG, "%s enable bluedroid failed: %s", __func__, esp_err_to_name(ret)); return ret; }

    // Registrar Callbacks
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){ ESP_LOGE(GATTS_TAG, "gatts register failed: %x", ret); return ret; }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){ ESP_LOGE(GATTS_TAG, "gap register failed: %x", ret); return ret; }

    // Registrar App Profile
    ret = esp_ble_gatts_app_register(PROFILE_APP_ID);
    if (ret){ ESP_LOGE(GATTS_TAG, "gatts app register failed: %x", ret); return ret; }

    // Setear MTU local
    ret = esp_ble_gatt_set_local_mtu(200); // O hasta 517
    if (ret){ ESP_LOGE(GATTS_TAG, "set local MTU failed: %x", ret); } // No retornar en fallo de MTU

    // Iniciar Tarea de Simulación de Batería
    BaseType_t task_created = xTaskCreate(battery_simulation_task, "BatSim", 2048, NULL, 5, NULL);
    if (task_created != pdPASS) {
        ESP_LOGE(GATTS_TAG, "Error creando tarea de simulación de batería!");
        // ¿Retornar error aquí? Depende de cuán crítico sea.
    }

    ESP_LOGI(GATTS_TAG, "BLE VibeBrain Service Initialized successfully.");
    return ESP_OK;
}