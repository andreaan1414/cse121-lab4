#pragma once

#include <stdbool.h>

bool icm_init(void);
bool icm_read_accel(float *x, float *y, float *z);

