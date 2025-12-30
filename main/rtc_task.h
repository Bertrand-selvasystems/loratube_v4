#pragma once

#include "esp_err.h"
#include "pcf8523_module.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    pcf8523_t *rtc;
} rtc_task_cfg_t;

esp_err_t rtc_task_start(const rtc_task_cfg_t *cfg);

#ifdef __cplusplus
}
#endif
