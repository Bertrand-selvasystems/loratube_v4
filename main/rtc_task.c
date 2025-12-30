#include "rtc_task.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "system_state.h"
#include "system_queues.h"

static const char *TAG = "RTC_TASK";

// Alarm field disable bit (common NXP scheme): bit7=1 disables compare for that field
#define ALARM_AE_BIT (1u << 7)
static inline uint8_t bin2bcd_(uint8_t v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }

static esp_err_t rtc_ping_(pcf8523_t *rtc)
{
    uint8_t v = 0;
    return pcf8523_read_reg(rtc, PCF8523_REG_CONTROL_1, &v);
}

static esp_err_t rtc_alarm_is_daily_midnight_(pcf8523_t *rtc, bool *ok)
{
    if (!rtc || !ok) return ESP_ERR_INVALID_ARG;

    uint8_t a[4] = {0};
    esp_err_t ret = pcf8523_read(rtc, PCF8523_REG_MIN_ALARM, a, sizeof(a));
    if (ret != ESP_OK) return ret;

    const bool min_ok  = ((a[0] & ALARM_AE_BIT) == 0) && ((a[0] & 0x7F) == bin2bcd_(0));
    const bool hour_ok = ((a[1] & ALARM_AE_BIT) == 0) && ((a[1] & 0x7F) == bin2bcd_(0));
    const bool day_dis = ((a[2] & ALARM_AE_BIT) != 0);
    const bool wd_dis  = ((a[3] & ALARM_AE_BIT) != 0);

    *ok = (min_ok && hour_ok && day_dis && wd_dis);
    return ESP_OK;
}

static esp_err_t rtc_program_daily_midnight_alarm_(pcf8523_t *rtc)
{
    // minute=00 enabled, hour=00 enabled, day/wday disabled => triggers every day at 00:00
    uint8_t a[4];
    a[0] = (uint8_t)(bin2bcd_(0) & 0x7F);   // min, AE=0
    a[1] = (uint8_t)(bin2bcd_(0) & 0x7F);   // hour, AE=0
    a[2] = (uint8_t)(ALARM_AE_BIT);         // day disabled
    a[3] = (uint8_t)(ALARM_AE_BIT);         // weekday disabled

    esp_err_t ret = pcf8523_write(rtc, PCF8523_REG_MIN_ALARM, a, sizeof(a));
    if (ret != ESP_OK) return ret;

    // Enable Alarm interrupt in CONTROL_1 (AIE)
    uint8_t c1 = 0;
    ret = pcf8523_read_reg(rtc, PCF8523_REG_CONTROL_1, &c1);
    if (ret != ESP_OK) return ret;

    c1 |= PCF8523_CTRL1_AIE;
    ret = pcf8523_write_reg(rtc, PCF8523_REG_CONTROL_1, c1);
    if (ret != ESP_OK) return ret;

    // Clear AF so INT can be released/armed cleanly for next day
    (void)pcf8523_clear_flags(rtc, PCF8523_CTRL2_AF, false);

    return ESP_OK;
}

static void rtc_task_main_(void *arg)
{
    rtc_task_cfg_t cfg = {0};
    memcpy(&cfg, arg, sizeof(cfg));
    vPortFree(arg);

    // By default, do not claim DATE_VALID until we successfully publish a datetime
    state_clear(EGS_DATE_VALID);

    // 1) Ping
    esp_err_t ret = rtc_ping_(cfg.rtc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RTC ping failed: %s", esp_err_to_name(ret));
        state_set(EGS_ERR_RTC);
        vTaskDelete(NULL);
        return;
    }
    state_clear(EGS_ERR_RTC);

    // 2) Ensure alarm daily midnight
    bool alarm_ok = false;
    ret = rtc_alarm_is_daily_midnight_(cfg.rtc, &alarm_ok);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Alarm check failed: %s", esp_err_to_name(ret));
        state_set(EGS_ERR_RTC);
        vTaskDelete(NULL);
        return;
    }

    if (!alarm_ok) {
        ESP_LOGW(TAG, "Programming daily midnight alarm");
        ret = rtc_program_daily_midnight_alarm_(cfg.rtc);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Alarm program failed: %s", esp_err_to_name(ret));
            state_set(EGS_ERR_RTC);
            vTaskDelete(NULL);
            return;
        }
    } else {
        ESP_LOGI(TAG, "Daily midnight alarm already configured");
    }

    // 3) Read datetime and publish
    pcf8523_datetime_t dt = {0};
    ret = pcf8523_get_datetime(cfg.rtc, &dt);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Get datetime failed: %s", esp_err_to_name(ret));
        state_set(EGS_ERR_RTC);
        vTaskDelete(NULL);
        return;
    }

    if (q_rtc_time) {
        rtc_time_msg_t m = {
            .year  = dt.year,
            .month = dt.month,
            .day   = dt.day,
            .hour  = dt.hour,
            .min   = dt.min,
            .sec   = dt.sec,
            .wday  = dt.wday,
        };
        xQueueOverwrite(q_rtc_time, &m);
        state_set(EGS_DATE_VALID);
    } else {
        ESP_LOGW(TAG, "q_rtc_time is NULL (system_queues_init missing?)");
        // If queue missing, date isn't "valid" for the system
        state_clear(EGS_DATE_VALID);
    }

    // 4) No explicit DONE bit in your design (good).
    // The stable proof is EGS_DATE_VALID and absence of EGS_ERR_RTC.

    // 5) Self delete
    vTaskDelete(NULL);
}

static pcf8523_t *g_rtc = NULL;
static TaskHandle_t g_task = NULL;

esp_err_t rtc_task_start(const rtc_task_cfg_t *cfg)
{
    if (!cfg || !cfg->rtc) return ESP_ERR_INVALID_ARG;
    if (!cfg->rtc->initialized) return ESP_ERR_INVALID_STATE;
    if (g_task) return ESP_ERR_INVALID_STATE;

    g_rtc = cfg->rtc;

    BaseType_t ok = xTaskCreate(rtc_task_main_, "rtc_task", 768, NULL,
                               tskIDLE_PRIORITY + 1, &g_task);
    if (ok != pdPASS) { g_task = NULL; g_rtc = NULL; return ESP_ERR_NO_MEM; }
    return ESP_OK;
}