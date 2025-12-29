#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

// Dépendances modules
#include "PCA9536_module.h"
#include "PCF8523_module.h"
#include "E22_module.h"
#include "FRAM_module.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // --- I2C bus ---
    i2c_port_t i2c_port;
    gpio_num_t sda_gpio;
    gpio_num_t scl_gpio;
    uint32_t   i2c_freq_hz;
    uint32_t   i2c_timeout_ms;

    // --- I2C addresses ---
    uint8_t pca9536_addr;
    uint8_t pcf8523_addr;
    uint8_t fram_addr;

    // --- E22 config ---
    e22_config_t e22_cfg;

    // --- comportement init ---
    bool     do_rtc_debug_irq_timer;
    bool     force_rtc_sqw_hiz;
    bool     blink_leds;
    uint8_t  blink_cycles;
    uint32_t blink_period_ms;

    bool enable_e22_power;   // GP0
    bool enable_buck_mode;   // GP3
} loratube_init_config_t;

typedef struct {
    // Handles / drivers
    pca9536_handle_t io;
    pcf8523_t        rtc;
    e22_handle_t     e22;

    // FRAM
    fram_meta_t fram_meta;
    bool        fram_ready;

    // Copie config utile
    loratube_init_config_t cfg;

    bool i2c_ready;
} loratube_ctx_t;

// Config par défaut
loratube_init_config_t loratube_init_config_default(void);

// Init / deinit
esp_err_t loratube_init(loratube_ctx_t *ctx, const loratube_init_config_t *cfg);
esp_err_t loratube_deinit(loratube_ctx_t *ctx);

// Helpers PCA
esp_err_t loratube_led_green(loratube_ctx_t *ctx, bool on);
esp_err_t loratube_led_red(loratube_ctx_t *ctx, bool on);
esp_err_t loratube_e22_power(loratube_ctx_t *ctx, bool on);
esp_err_t loratube_buck_mode(loratube_ctx_t *ctx, bool on);

// Smoke test E22
esp_err_t loratube_e22_smoke_test(loratube_ctx_t *ctx);

// FRAM init/load
esp_err_t loratube_fram_init_and_load(loratube_ctx_t *ctx, uint8_t fram_i2c_addr);

#ifdef __cplusplus
}
#endif
