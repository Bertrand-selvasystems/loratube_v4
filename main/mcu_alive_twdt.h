#pragma once
#include <stdint.h>
#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t twdt_timeout_ms;   // ex: 8000
    uint32_t kick_period_ms;    // ex: 1000
    uint32_t idle_core_mask;    // ex: (1<<0) on ESP32-C3
    bool     trigger_panic;     // true = reset hard via panic
} sw_wdt_cfg_t;

/**
 * @brief Initialize Task Watchdog and start a low-priority kicker task.
 *
 * Notes:
 * - This module does NOT do per-task heartbeat.
 * - Use idle_core_mask to watch the idle task => prevents false safety when CPU is stuck busy.
 */
esp_err_t sw_wdt_start(const sw_wdt_cfg_t *cfg);

#ifdef __cplusplus
}
#endif
