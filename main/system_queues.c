#include "system_queues.h"

#include "esp_log.h"

static const char *TAG = "QUEUES";

QueueHandle_t q_temp = NULL;
QueueHandle_t q_vbat = NULL;
QueueHandle_t q_rtc_time = NULL;

esp_err_t system_queues_init(void)
{
    if (q_temp || q_vbat || q_rtc_time) return ESP_ERR_INVALID_STATE;

    q_temp = xQueueCreate(1, sizeof(c3_temp_msg_t));
    if (!q_temp) goto oom;

    q_vbat = xQueueCreate(1, sizeof(c3_vbat_msg_t));
    if (!q_vbat) goto oom;

    q_rtc_time = xQueueCreate(1, sizeof(rtc_time_msg_t));
    if (!q_rtc_time) goto oom;

    system_queues_dump();
    return ESP_OK;

oom:
    system_queues_deinit();
    return ESP_ERR_NO_MEM;
}

void system_queues_deinit(void)
{
    if (q_temp) { vQueueDelete(q_temp); q_temp = NULL; }
    if (q_vbat) { vQueueDelete(q_vbat); q_vbat = NULL; }
    if (q_rtc_time) { vQueueDelete(q_rtc_time); q_rtc_time = NULL; }
}

void system_queues_dump(void)
{
    ESP_LOGI(TAG, "q_temp=%p item=%u depth=1", (void*)q_temp, (unsigned)sizeof(c3_temp_msg_t));
    ESP_LOGI(TAG, "q_vbat=%p item=%u depth=1", (void*)q_vbat, (unsigned)sizeof(c3_vbat_msg_t));
    ESP_LOGI(TAG, "q_rtc_time=%p item=%u depth=1", (void*)q_rtc_time, (unsigned)sizeof(rtc_time_msg_t));
}