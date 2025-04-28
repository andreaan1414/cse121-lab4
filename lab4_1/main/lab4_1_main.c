// #include <stdio.h>
// #include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/i2c.h"
// #include "esp_log.h"

// #define I2C_MASTER_NUM I2C_NUM_0
// #define I2C_MASTER_SDA_IO 10
// #define I2C_MASTER_SCL_IO 8
// #define I2C_MASTER_FREQ_HZ 400000
// #define ICM42670_ADDR 0x68

// #define TAG "LAB4_1"

// // I2C initialization
// void i2c_master_init() {
//     i2c_config_t conf = {
//         .mode = I2C_MODE_MASTER,
//         .sda_io_num = I2C_MASTER_SDA_IO,
//         .scl_io_num = I2C_MASTER_SCL_IO,
//         .sda_pullup_en = GPIO_PULLUP_ENABLE,
//         .scl_pullup_en = GPIO_PULLUP_ENABLE,
//         .master.clk_speed = I2C_MASTER_FREQ_HZ,
//     };
//     i2c_param_config(I2C_MASTER_NUM, &conf);
//     i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
// }

// // Wake up the ICM42670 sensor
// void icm42670_wake() {
//     uint8_t wake_cmd[] = {0x1F, 0x03}; // 0x1F = PWR_MGMT0, 0x03 = Accel and Gyro ON
//     i2c_master_write_to_device(I2C_MASTER_NUM, ICM42670_ADDR, wake_cmd, 2, pdMS_TO_TICKS(100));
//     vTaskDelay(pdMS_TO_TICKS(10)); // Short delay to stabilize
// }

// // Read two bytes from sensor starting at reg_addr
// int16_t read_axis(uint8_t reg_addr) {
//     uint8_t data[2];
//     esp_err_t err = i2c_master_write_read_device(
//         I2C_MASTER_NUM, ICM42670_ADDR, &reg_addr, 1, data, 2, pdMS_TO_TICKS(100)
//     );
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "I2C Read Failed: %s", esp_err_to_name(err));
//         return 0;
//     }
//     return (int16_t)((data[0] << 8) | data[1]);
// }

// // Detect tilt and print UP/DOWN/LEFT/RIGHT
// void detect_tilt_and_log() {
//     int16_t accel_x = read_axis(0x1F); // ACCEL_XOUT_H
//     int16_t accel_y = read_axis(0x21); // ACCEL_YOUT_H

//     const char* dir_x = "";
//     const char* dir_y = "";

//     if (accel_x > 3000) {
//         dir_x = "RIGHT";
//     } else if (accel_x < -3000) {
//         dir_x = "LEFT";
//     }

//     if (accel_y > 3000) {
//         dir_y = "UP";
//     } else if (accel_y < -3000) {
//         dir_y = "DOWN";
//     }

//     if (strlen(dir_x) || strlen(dir_y)) {
//         ESP_LOGI(TAG, "%s %s", dir_y, dir_x);
//     }
// }

// void app_main(void) {
//     i2c_master_init();
//     icm42670_wake();

//     while (1) {
//         detect_tilt_and_log();
//         vTaskDelay(pdMS_TO_TICKS(300)); // Check every 300ms
//     }
// }
// worked but was a little finicky

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define LOG_TAG "LAB4_1"
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 10
#define I2C_MASTER_SCL_IO 8
#define I2C_MASTER_FREQ_HZ 400000
#define ICM42670_ADDR 0x68

// I2C master init
void i2c_master_init(void)
{
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

// Wake ICM42670
void icm42670_wake(void)
{
    uint8_t wake_cmd[] = {0x1F, 0x03}; // PWR_MGMT0 = 0x03 to enable accel + gyro
    esp_err_t err = i2c_master_write_to_device(I2C_MASTER_NUM, ICM42670_ADDR, wake_cmd, sizeof(wake_cmd), pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGE(LOG_TAG, "Sensor wake failed: %s", esp_err_to_name(err));
    }
    vTaskDelay(pdMS_TO_TICKS(10));
}

// Read axis data
int16_t read_axis(uint8_t reg_addr)
{
    uint8_t data[2];
    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, ICM42670_ADDR, &reg_addr, 1, data, 2, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGE(LOG_TAG, "I2C Read Failed: %s", esp_err_to_name(err));
        return 0;
    }
    return (int16_t)((data[0] << 8) | data[1]);
}

// Tilt detection task
void tilt_task(void *pvParams)
{
    while (1) {
        int16_t accel_x = read_axis(0x1F); // ACCEL_XOUT_H
        int16_t accel_y = read_axis(0x21); // ACCEL_YOUT_H

        const char *dir_x = "";
        const char *dir_y = "";

        if (accel_x > 3000) dir_x = "RIGHT";
        else if (accel_x < -3000) dir_x = "LEFT";

        if (accel_y > 3000) dir_y = "UP";
        else if (accel_y < -3000) dir_y = "DOWN";

        if (strlen(dir_x) || strlen(dir_y)) {
            ESP_LOGI(LOG_TAG, "Tilt Detected: %s %s", dir_y, dir_x);
        }

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

void app_main(void)
{
    ESP_LOGI(LOG_TAG, "Starting Lab 4.1 - Tilt Detection");

    i2c_master_init();
    icm42670_wake();

    xTaskCreate(tilt_task, "tilt_task", 2048, NULL, 5, NULL);
}
