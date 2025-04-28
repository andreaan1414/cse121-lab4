#pragma once

#include "esp_err.h"
#include "esp_hidd_api.h"

esp_err_t esp_hidd_profile_init(void);
void esp_hidd_profile_deinit(void);
