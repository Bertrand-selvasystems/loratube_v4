#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "system_types.h" 


#ifdef __cplusplus
extern "C" {
#endif

// =====================
// Queues globales
// =====================
extern QueueHandle_t q_temp;
extern QueueHandle_t q_vbat;
extern QueueHandle_t q_rtc_time;

// =====================
// Init / Debug
// =====================
esp_err_t system_queues_init(void);
void system_queues_deinit(void);
void system_queues_dump(void);

#ifdef __cplusplus
}
#endif
