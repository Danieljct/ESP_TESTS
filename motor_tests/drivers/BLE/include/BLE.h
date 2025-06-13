#ifndef _BLE_H_
#define _BLE_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"

#include "pin_config.h"
#include "sdkconfig.h"
#include "signal.h"
#include "max17055.h"

//#include "rgb.h"
esp_err_t gatt_app_init();

// --- Estructura para las características ---
typedef struct {
    uint32_t period;       // Valor inicial de ejemplo (ms)
    uint32_t repetitions;  // Valor inicial de ejemplo
    bool play_stop;        // Valor inicial de ejemplo
    uint8_t duty_cycle;    // Valor inicial de ejemplo (0-100)
} vibebrain_characteristics_t;

void ble_vibebrain_update_repetitions(void);
esp_err_t ble_vibebrain_init(void);

#endif