#include "icm42670p.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>

#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_IO      10
#define I2C_SCL_IO      8
#define I2C_FREQ_HZ     400000

#define ICM_ADDR        0x70 // used ot be 68 

static const char *TAG = "ICM42670P";

bool icm_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };

    ESP_LOGI(TAG, "Configuring I2C...");
    esp_err_t ret;

    ret = i2c_param_config(I2C_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(ret));
        return false;
    }

    //  Send soft reset
    uint8_t reset_cmd[] = {0x11, 0x01};
    ret = i2c_master_write_to_device(I2C_PORT, ICM_ADDR, reset_cmd, 2, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Soft reset failed: %s", esp_err_to_name(ret));
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for reset to complete

    //  Read WHO_AM_I
    uint8_t reg = 0x75;
    uint8_t id = 0;
    ret = i2c_master_write_read_device(I2C_PORT, ICM_ADDR, &reg, 1, &id, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I read failed: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "WHO_AM_I register value: 0x%02X", id);
    return true;
}


bool icm_read_accel(float *x, float *y, float *z) {
    uint8_t accel_reg = 0x1D; // ACCEL_XOUT_H
    uint8_t data[6];

    esp_err_t ret = i2c_master_write_read_device(I2C_PORT, ICM_ADDR, &accel_reg, 1, data, 6, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Accel read failed: %s", esp_err_to_name(ret));
        return false;
    }

    int16_t ax = (data[0] << 8) | data[1];
    int16_t ay = (data[2] << 8) | data[3];
    int16_t az = (data[4] << 8) | data[5];

    *x = ax / 16384.0f;
    *y = ay / 16384.0f;
    *z = az / 16384.0f;

    return true;
}

        
