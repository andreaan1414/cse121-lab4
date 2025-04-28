#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/i2c.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_hidd_prf_api.h"
#include "hid_dev.h"

#define HID_TAG "HID_MOUSE"
#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_SCL_IO      8
#define I2C_MASTER_SDA_IO      10
#define I2C_MASTER_FREQ_HZ     400000
#define ICM42670_ADDR          0x68

static uint16_t hid_conn_id = 0;
static bool sec_conn = false;

// ======================== I2C + ACCEL ==========================

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

void icm42670_wake() {
    uint8_t wake_cmd[] = {0x1F, 0x03}; // Power on accel + gyro
    i2c_master_write_to_device(I2C_MASTER_NUM, ICM42670_ADDR, wake_cmd, sizeof(wake_cmd), pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(10));
}

int16_t read_axis(uint8_t reg_addr) {
    uint8_t data[2];
    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, ICM42670_ADDR, &reg_addr, 1, data, 2, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGE(HID_TAG, "I2C Read Failed: %s", esp_err_to_name(err));
        return 0;
    }
    return (int16_t)((data[0] << 8) | data[1]);
}

// ======================== BLE STUFF ==========================

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch (event) {
        case ESP_HIDD_EVENT_REG_FINISH:
            if (param->init_finish.state == ESP_HIDD_INIT_OK) {
                esp_ble_gap_set_device_name("ESP32_MOUSE");
                esp_ble_gap_config_adv_data(&hidd_adv_data);
            }
            break;
        case ESP_HIDD_EVENT_BLE_CONNECT:
            ESP_LOGI(HID_TAG, "BLE connected");
            hid_conn_id = param->connect.conn_id;
            break;
        case ESP_HIDD_EVENT_BLE_DISCONNECT:
            sec_conn = false;
            ESP_LOGI(HID_TAG, "BLE disconnected");
            esp_ble_gap_start_advertising(&hidd_adv_params);
            break;
        default:
            break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    if (event == ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT) {
        esp_ble_gap_start_advertising(&hidd_adv_params);
    } else if (event == ESP_GAP_BLE_AUTH_CMPL_EVT) {
        sec_conn = param->ble_security.auth_cmpl.success;
    }
}

esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .appearance = 0x03C2, // Mouse
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .service_uuid_len = 16,
    .p_service_uuid = (uint8_t[]){
        0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
        0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
    },
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x30,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// ======================== MOUSE TASK ==========================

void mouse_task(void *pvParameters)
{
    i2c_master_init();
    icm42670_wake();
    vTaskDelay(pdMS_TO_TICKS(100));

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));

        if (!sec_conn) continue;

        int16_t x = read_axis(0x1F); // ACCEL_XOUT_H
        int8_t delta_x = 0;

        if (x > 3000) delta_x = 10;     // Move right
        else if (x < -3000) delta_x = -10; // Move left

        if (delta_x != 0) {
            ESP_LOGI(HID_TAG, "Tilt x=%d -> move x=%d", x, delta_x);
            esp_hidd_send_mouse_value(hid_conn_id, 0, delta_x, 0, 0, 0);
        }
    }
}

// ======================== MAIN ==========================

void app_main(void)
{
    ESP_LOGI(HID_TAG, "Starting BLE Mouse with Tilt");

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_hidd_profile_init());
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);

    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(iocap));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(key_size));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(init_key));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(rsp_key));

    xTaskCreate(mouse_task, "mouse_task", 4096, NULL, 5, NULL);
}

