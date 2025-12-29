#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "PCA9536_module.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PCA_LED_OFF        = 0,
    PCA_LED_ON         = 1,
    PCA_LED_BLINK_SLOW = 2,
    PCA_LED_BLINK_FAST = 3,
} pca_led_mode_t;

/**
 * @brief Initialize PCA manager (creates queue and starts the task).
 *
 * Must be called once, after pca9536_init().
 */
esp_err_t pca_mgr_init(pca9536_handle_t *pca);

/** @brief Set GREEN LED mode (patch message). */
esp_err_t pca_mgr_set_green(pca_led_mode_t mode);

/** @brief Set RED LED mode (patch message). */
esp_err_t pca_mgr_set_red(pca_led_mode_t mode);

/** @brief Trigger E22 reset pulse (forces E22_EN=0 for E22_PULSE_MS). */
esp_err_t pca_mgr_pulse_e22_reset(void);

/** @brief Force buck mode ON (latched true until cleared at sleep or reset). */
esp_err_t pca_mgr_force_buck_on(void);

#ifdef __cplusplus
}
#endif
