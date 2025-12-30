/**
 * @file sw_wdt_task.c
 * @brief Low priority task that periodically kicks the ESP-IDF Task Watchdog (TWDT).
 *
 * Philosophy (V4):
 * - No heartbeat bits.
 * - No polling of peripherals.
 * - Just keep a software watchdog armed as a "deadlock / runaway code" safety net.
 *
 * IMPORTANT:
 * - Watch the idle task (idle_core_mask) so that a CPU hog / scheduler starvation still triggers reset.
 */

#include "mcu_alive_twdt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_task_wdt.h"

static const char *TAG = "SW_WDT";

#ifndef SW_WDT_STACK
#define SW_WDT_STACK 2048
#endif

#ifndef SW_WDT_TASK_PRIO
#define SW_WDT_TASK_PRIO (tskIDLE_PRIORITY)   // ultra low
#endif

static uint32_t g_kick_period_ms = 1000;

static void sw_wdt_kicker_task_(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "kicker started: period=%u ms", (unsigned)g_kick_period_ms);

    /* Add THIS task to TWDT, so if it freezes it can trigger too.
     * Not strictly necessary if idle is watched, but harmless.
     */
    esp_err_t e = esp_task_wdt_add(NULL);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_task_wdt_add(kicker) failed: %s", esp_err_to_name(e));
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(g_kick_period_ms));
        /* Kick TWDT (best-effort) */
        (void)esp_task_wdt_reset();
    }
}

esp_err_t sw_wdt_start(const sw_wdt_cfg_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    if (cfg->twdt_timeout_ms < 1000) return ESP_ERR_INVALID_ARG;
    if (cfg->kick_period_ms == 0 || cfg->kick_period_ms >= cfg->twdt_timeout_ms)
        return ESP_ERR_INVALID_ARG;

    g_kick_period_ms = cfg->kick_period_ms;

    /* Init TWDT (ESP-IDF v5+) */
    const esp_task_wdt_config_t twdt_cfg = {
        .timeout_ms = cfg->twdt_timeout_ms,
        .idle_core_mask = cfg->idle_core_mask,
        .trigger_panic = cfg->trigger_panic,
    };

    esp_err_t e = esp_task_wdt_init(&twdt_cfg);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_task_wdt_init failed: %s", esp_err_to_name(e));
        return e;
    }

    /* Start kicker task */
    BaseType_t ok = xTaskCreate(
        sw_wdt_kicker_task_,
        "sw_wdt",
        SW_WDT_STACK,
        NULL,
        SW_WDT_TASK_PRIO,
        NULL
    );
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate(sw_wdt) failed");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "TWDT started: timeout=%u ms kick=%u ms idle_mask=0x%lx panic=%d",
             (unsigned)cfg->twdt_timeout_ms,
             (unsigned)cfg->kick_period_ms,
             (unsigned long)cfg->idle_core_mask,
             (int)cfg->trigger_panic);

    return ESP_OK;
}
