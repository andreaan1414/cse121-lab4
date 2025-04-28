#include "hidd_le_prf_int.h"
#include "esp_log.h"

static const char *TAG = "HID_LE_PRF";

void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param) {
    switch (event) {
        case ESP_HIDD_EVENT_REG_FINISH:
            ESP_LOGI(TAG, "HID Service Registered");
            break;
        case ESP_HIDD_EVENT_BLE_CONNECT:
            ESP_LOGI(TAG, "Device Connected");
            break;
        case ESP_HIDD_EVENT_BLE_DISCONNECT:
            ESP_LOGI(TAG, "Device Disconnected");
            break;
        default:
            break;
    }
}
