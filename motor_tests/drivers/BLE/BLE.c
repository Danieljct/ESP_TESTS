#include "BLE.h"
#include "esp_log.h"
#include "string.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatt_defs.h" // Para ESP_GATT_UUID_CHAR_CLIENT_CONFIG, etc.
#include "nvs_flash.h"     // Para nvs_flash_init, ESP_ERR_NVS_*

// Definiciones de UUIDs para nuestro servicio de comandos
#define GATTS_TAG_BLE "CMD_GATTS" // Tag para los logs BLE

// Se usan directamente los defines de ble.h
static const uint16_t GATTS_CHAR_UUID_CMD_INTERNAL = GATTS_CHAR_UUID_COMMAND;
static const uint16_t GATTS_CHAR_UUID_RSP_INTERNAL = GATTS_CHAR_UUID_RESPONSE;

// --- Instancia del Perfil para nuestro servicio de comandos ---
#define PROFILE_NUM_BLE 1
#define PROFILE_APP_ID_BLE 0

static ble_profile_data_t gl_ble_profile_tab[PROFILE_NUM_BLE] = {
    [PROFILE_APP_ID_BLE] = {
        .gatts_if = ESP_GATT_IF_NONE,
        .is_connected = false,
        .response_notify_enabled = false,
        .commands_service_handle = 0,
        .command_char_handle = 0,
        .response_char_handle = 0,
        .response_descr_handle = 0,
    }
};

// Cola para comandos BLE
QueueHandle_t ble_command_queue;

// Variables para el advertising (adaptadas del ejemplo)
static char ble_device_name[ESP_BLE_ADV_NAME_LEN_MAX] = "ESP32_DRV2605";
static uint8_t adv_config_done = 0;
#define ADV_CONFIG_FLAG     (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

static uint8_t adv_service_uuid128[16] = { /* LSB <--------> MSB */
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00,
    LO_BYTE(GATTS_SERVICE_UUID_COMMANDS), HI_BYTE(GATTS_SERVICE_UUID_COMMANDS), 0x00, 0x00,
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006, // 3.75ms
    .max_interval = 0x0010, // 10ms
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20, // 12.5ms
    .adv_int_max        = 0x40, // 25ms
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


// Prototipo de la función de evento GATTS para el perfil de comandos
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

// Define una estructura para el service ID para nuestro servicio de comandos
static esp_gatt_srvc_id_t commands_service_id = {
    .id.uuid.len = ESP_UUID_LEN_16,
    .id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_COMMANDS,
    .is_primary = true,
};


// Manejador de eventos GAP
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG_BLE, "Advertising start failed");
        } else {
            ESP_LOGI(GATTS_TAG_BLE, "Advertising start successfully");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG_BLE, "Advertising stop failed");
        } else {
            ESP_LOGI(GATTS_TAG_BLE, "Advertising stop successfully");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(GATTS_TAG_BLE, "update con params status = %d, min_int = %d, max_int = %d, conn_int = %d, latency = %d, timeout = %d",
                 param->update_conn_params.status,
                 param->update_conn_params.min_int,
                 param->update_conn_params.max_int,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

// Manejador de eventos GATTS para el perfil de comandos
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG_BLE, "REGISTER_APP_EVT, status %d, app_id %d, gatts_if %d", param->reg.status, param->reg.app_id, gatts_if);
        if (param->reg.status == ESP_GATT_OK) {
            gl_ble_profile_tab[PROFILE_APP_ID_BLE].gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATTS_TAG_BLE, "Reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
            return;
        }

        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(ble_device_name);
        if (set_dev_name_ret) ESP_LOGE(GATTS_TAG_BLE, "set device name failed, error code = %x", set_dev_name_ret);
        esp_err_t ret_adv = esp_ble_gap_config_adv_data(&adv_data);
        if (ret_adv) ESP_LOGE(GATTS_TAG_BLE, "config adv data failed, error code = %x", ret_adv);
        adv_config_done |= ADV_CONFIG_FLAG;
        esp_err_t ret_scan_rsp = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret_scan_rsp) ESP_LOGE(GATTS_TAG_BLE, "config scan response data failed, error code = %x", ret_scan_rsp);
        adv_config_done |= SCAN_RSP_CONFIG_FLAG;

        // Crear Servicio de Comandos
        esp_ble_gatts_create_service(gatts_if, &commands_service_id, 10);
        break;

    case ESP_GATTS_CREATE_EVT: {
        // CORREGIDO: Acceso a uuid16 directamente del union en esp_bt_uuid_t
        // En CREATE_EVT

        if (param->create.status == ESP_GATT_OK) {
            uint16_t created_service_handle = param->create.service_handle;
            gl_ble_profile_tab[PROFILE_APP_ID_BLE].commands_service_handle = created_service_handle;
            esp_ble_gatts_start_service(created_service_handle);

            // Añadir característica de COMANDO (WRITE)
            esp_ble_gatts_add_char(created_service_handle,
                                   &(esp_bt_uuid_t){.len=ESP_UUID_LEN_16, .uuid.uuid16=GATTS_CHAR_UUID_CMD_INTERNAL},
                                   ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_WRITE,
                                   NULL, NULL);

            // Añadir característica de RESPUESTA (READ, NOTIFY)
            esp_ble_gatts_add_char(created_service_handle,
                                   &(esp_bt_uuid_t){.len=ESP_UUID_LEN_16, .uuid.uuid16=GATTS_CHAR_UUID_RSP_INTERNAL},
                                   ESP_GATT_PERM_READ,
                                   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                   NULL, NULL);
        } else {
            ESP_LOGE(GATTS_TAG_BLE, "Create service failed, status %d", param->create.status);
        }
        break;
    }

    case ESP_GATTS_ADD_CHAR_EVT: {
        uint16_t attr_handle = param->add_char.attr_handle;
        uint16_t service_handle = param->add_char.service_handle;
        // CORREGIDO: Acceso a uuid16 directamente del union en esp_bt_uuid_t
        uint16_t char_uuid = param->add_char.char_uuid.uuid.uuid16;

        ESP_LOGI(GATTS_TAG_BLE, "ADD_CHAR_EVT, status %d, attr_handle %d, service_handle %d, char_uuid: 0x%04X",
                 param->add_char.status, attr_handle, service_handle, char_uuid);

        if (param->add_char.status != ESP_GATT_OK) {
            ESP_LOGE(GATTS_TAG_BLE, "Failed to add characteristic, UUID=0x%04X, status=0x%X", char_uuid, param->add_char.status);
            break;
        }

        if (service_handle == gl_ble_profile_tab[PROFILE_APP_ID_BLE].commands_service_handle) {
            if (char_uuid == GATTS_CHAR_UUID_CMD_INTERNAL) {
                gl_ble_profile_tab[PROFILE_APP_ID_BLE].command_char_handle = attr_handle;
                ESP_LOGI(GATTS_TAG_BLE, "Command Char (0x%04X) handle stored: %d", char_uuid, attr_handle);
            } else if (char_uuid == GATTS_CHAR_UUID_RSP_INTERNAL) {
                gl_ble_profile_tab[PROFILE_APP_ID_BLE].response_char_handle = attr_handle;
                ESP_LOGI(GATTS_TAG_BLE, "Response Char (0x%04X) handle stored: %d", char_uuid, attr_handle);

                ESP_LOGI(GATTS_TAG_BLE, "Adding CCCD for Response Char 0x%04X", char_uuid);
                esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(
                                            service_handle,
                                            &(esp_bt_uuid_t){.len=ESP_UUID_LEN_16, .uuid.uuid16=ESP_GATT_UUID_CHAR_CLIENT_CONFIG},
                                            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                            NULL, NULL);
                if (add_descr_ret) {
                    ESP_LOGE(GATTS_TAG_BLE, "add char descr failed, error code = 0x%x", add_descr_ret);
                }
            }
        }
        break;
    }

    case ESP_GATTS_ADD_CHAR_DESCR_EVT: {
        uint16_t service_handle = param->add_char_descr.service_handle;
        uint16_t descr_handle = param->add_char_descr.attr_handle;
        // CORREGIDO: Acceso a uuid16 directamente del union en esp_bt_uuid_t
        uint16_t descr_uuid = param->add_char_descr.descr_uuid.uuid.uuid16;

        ESP_LOGI(GATTS_TAG_BLE, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d, descr_uuid: 0x%04X",
                 param->add_char_descr.status, descr_handle, service_handle, descr_uuid);

        if (param->add_char_descr.status != ESP_GATT_OK) {
            ESP_LOGE(GATTS_TAG_BLE, "Failed to add descriptor, UUID=0x%04X, status=0x%X", descr_uuid, param->add_char_descr.status);
            break;
        }

        if (descr_uuid == ESP_GATT_UUID_CHAR_CLIENT_CONFIG &&
            service_handle == gl_ble_profile_tab[PROFILE_APP_ID_BLE].commands_service_handle) {
            gl_ble_profile_tab[PROFILE_APP_ID_BLE].response_descr_handle = descr_handle;
            ESP_LOGI(GATTS_TAG_BLE, "Response Char CCCD handle stored: %d", descr_handle);
        }
        break;
    }

    case ESP_GATTS_READ_EVT: {
        ESP_LOGD(GATTS_TAG_BLE, "GATT_READ_EVT, handle %d", param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        esp_gatt_status_t status = ESP_GATT_OK;

        if (param->read.handle == gl_ble_profile_tab[PROFILE_APP_ID_BLE].response_char_handle) {
            rsp.attr_value.len = 0;
            status = ESP_GATT_OK;
        } else {
            ESP_LOGW(GATTS_TAG_BLE, "Read request for unknown handle: %d", param->read.handle);
            status = ESP_GATT_READ_NOT_PERMIT;
        }
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, status, &rsp);
        break;
    }

    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(GATTS_TAG_BLE, "GATT_WRITE_EVT, handle %d, len %d, is_prep %d", param->write.handle, param->write.len, param->write.is_prep);

        esp_gatt_status_t status = ESP_GATT_OK;

        if (param->write.handle == gl_ble_profile_tab[PROFILE_APP_ID_BLE].response_descr_handle) {
            if (param->write.len == 2) {
                uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                gl_ble_profile_tab[PROFILE_APP_ID_BLE].response_notify_enabled = (descr_value == 0x0001);
                ESP_LOGI(GATTS_TAG_BLE, "Response notify %s", gl_ble_profile_tab[PROFILE_APP_ID_BLE].response_notify_enabled ? "enabled" : "disabled");
            } else {
                status = ESP_GATT_INVALID_ATTR_LEN;
            }
        }
        else if (param->write.handle == gl_ble_profile_tab[PROFILE_APP_ID_BLE].command_char_handle) {
            if (param->write.len > 0 && param->write.len < BLE_MAX_COMMAND_LEN) {
                ble_command_t new_command;
                memset(&new_command, 0, sizeof(ble_command_t));
                memcpy(new_command.command, param->write.value, param->write.len);
                new_command.command[param->write.len] = '\0';

                if (xQueueSend(ble_command_queue, &new_command, portMAX_DELAY) != pdTRUE) {
                    ESP_LOGE(GATTS_TAG_BLE, "Fallo al enviar comando a la cola.");
                    status = ESP_GATT_ERROR;
                } else {
                    ESP_LOGI(GATTS_TAG_BLE, "Comando recibido y enviado a cola: %s", new_command.command);
                }
            } else {
                ESP_LOGE(GATTS_TAG_BLE, "Comando recibido es inválido o demasiado largo.");
                status = ESP_GATT_INVALID_ATTR_LEN;
            }
        }
        else {
            ESP_LOGW(GATTS_TAG_BLE, "Write EVT for unknown handle: %d", param->write.handle);
            status = ESP_GATT_WRITE_NOT_PERMIT;
        }

        if (param->write.is_prep) {
            ESP_LOGI(GATTS_TAG_BLE, "Prepare write received for handle %d, len %d", param->write.handle, param->write.len);
        }

        if (param->write.need_rsp){
            esp_err_t rsp_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
            if (rsp_err != ESP_OK) { ESP_LOGE(GATTS_TAG_BLE, "Error enviando write response: 0x%x", rsp_err); }
        }
        break;
    }

    case ESP_GATTS_CONNECT_EVT: {
        ESP_LOGI(GATTS_TAG_BLE, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote "ESP_BD_ADDR_STR"",
                 param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
        gl_ble_profile_tab[PROFILE_APP_ID_BLE].conn_id = param->connect.conn_id;
        gl_ble_profile_tab[PROFILE_APP_ID_BLE].is_connected = true;
        memcpy(gl_ble_profile_tab[PROFILE_APP_ID_BLE].remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        conn_params.latency = 0; conn_params.max_int = 0x20; conn_params.min_int = 0x10; conn_params.timeout = 400;
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    }

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG_BLE, "ESP_GATTS_DISCONNECT_EVT, conn_id %d, reason 0x%x", param->disconnect.conn_id, param->disconnect.reason);
        gl_ble_profile_tab[PROFILE_APP_ID_BLE].is_connected = false;
        gl_ble_profile_tab[PROFILE_APP_ID_BLE].response_notify_enabled = false;
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(GATTS_TAG_BLE, "ESP_GATTS_EXEC_WRITE_EVT");
        esp_ble_gatts_send_response(gatts_if, param->exec_write.conn_id, param->exec_write.trans_id, ESP_GATT_OK, NULL);
        break;

    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TAG_BLE, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;

    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG_BLE, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
        break;

    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TAG_BLE, "ESP_GATTS_CONF_EVT, status %d, handle %d", param->conf.status, param->conf.handle);
        break;

    case ESP_GATTS_UNREG_EVT:
    case ESP_GATTS_DELETE_EVT:
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    case ESP_GATTS_SET_ATTR_VAL_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_RESPONSE_EVT:
        ESP_LOGD(GATTS_TAG_BLE, "Unhandled GATTS Event: %d", event);
        break;

    default:
        break;
    }
}

// Manejador GATTS Principal (distribuye a los perfiles)
static void gatts_event_handler_main(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_ble_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATTS_TAG_BLE, "Reg app failed for app_id %d: %x", param->reg.app_id, param->reg.status);
            return;
        }
    }

    if (gatts_if == ESP_GATT_IF_NONE || gatts_if == gl_ble_profile_tab[PROFILE_APP_ID_BLE].gatts_if) {
        gatts_profile_event_handler(event, gatts_if, param);
    }
}

// Función para enviar respuestas al cliente BLE
void ble_send_response(const char *response) {
    if (!gl_ble_profile_tab[PROFILE_APP_ID_BLE].is_connected) {
        ESP_LOGW(GATTS_TAG_BLE, "No BLE client connected to send response.");
        return;
    }
    if (!gl_ble_profile_tab[PROFILE_APP_ID_BLE].response_notify_enabled) {
        ESP_LOGW(GATTS_TAG_BLE, "Response Characteristic notifications not enabled by client.");
        return;
    }

    uint16_t len = strlen(response);
    if (len > BLE_MAX_DATA_LEN) {
        len = BLE_MAX_DATA_LEN;
        ESP_LOGW(GATTS_TAG_BLE, "Response too long, truncating.");
    }

    esp_err_t err = esp_ble_gatts_send_indicate(
                        gl_ble_profile_tab[PROFILE_APP_ID_BLE].gatts_if,
                        gl_ble_profile_tab[PROFILE_APP_ID_BLE].conn_id,
                        gl_ble_profile_tab[PROFILE_APP_ID_BLE].response_char_handle,
                        len,
                        (uint8_t *)response,
                        false);
    if (err != ESP_OK) {
        ESP_LOGE(GATTS_TAG_BLE, "Error sending BLE response: 0x%x", err);
    } else {
        ESP_LOGI(GATTS_TAG_BLE, "BLE Response sent: %s", response);
    }
}

// Función Principal de Inicialización BLE
void ble_init(void) {
    esp_err_t ret;

    ble_command_queue = xQueueCreate(10, sizeof(ble_command_t));

    if (ble_command_queue == NULL) {
        ESP_LOGE(GATTS_TAG_BLE, "Fallo al crear la cola de comandos BLE");
        return;
    }

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) { ESP_LOGE(GATTS_TAG_BLE, "%s init controller failed: %s", __func__, esp_err_to_name(ret)); return; }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) { ESP_LOGE(GATTS_TAG_BLE, "%s enable controller failed: %s", __func__, esp_err_to_name(ret)); return; }

    ret = esp_bluedroid_init();
    if (ret) { ESP_LOGE(GATTS_TAG_BLE, "%s init bluedroid failed: %s", __func__, esp_err_to_name(ret)); return; }
    ret = esp_bluedroid_enable();
    if (ret) { ESP_LOGE(GATTS_TAG_BLE, "%s enable bluedroid failed: %s", __func__, esp_err_to_name(ret)); return; }

    ret = esp_ble_gatts_register_callback(gatts_event_handler_main);
    if (ret){ ESP_LOGE(GATTS_TAG_BLE, "gatts register failed: %x", ret); return; }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){ ESP_LOGE(GATTS_TAG_BLE, "gap register failed: %x", ret); return; }

    ret = esp_ble_gatts_app_register(PROFILE_APP_ID_BLE);
    if (ret){ ESP_LOGE(GATTS_TAG_BLE, "gatts app register failed: %x", ret); return; }

    ret = esp_ble_gatt_set_local_mtu(200);
    if (ret){ ESP_LOGE(GATTS_TAG_BLE, "set local MTU failed: %x", ret); }

    ESP_LOGI(GATTS_TAG_BLE, "BLE Command Service Initialized successfully.");
}