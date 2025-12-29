#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

// Contexte global init (I2C + handles PCA/RTC/E22)
#include "initialisation.h"

// ADC / sleep
#include "driver/adc.h"
#include "esp_sleep.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    // --- RTC tests ---
    bool rtc_debug_init_irq_timer;   // pcf8523_debug_init_irq_timer()
    bool rtc_force_sqw_hiz;          // pcf8523_make_sqw_hi_z()
    bool rtc_log_readback;           // log readback registers

    // --- PCA9536 tests ---
    bool blink_leds;
    uint8_t blink_cycles;
    uint32_t blink_period_ms;

    // --- Rails via PCA9536 ---
    bool set_e22_power_on;
    bool set_buck_mode_on;

    // --- E22 tests ---
    bool e22_smoke_test;
    bool e22_set_tx_power;
    uint8_t e22_tx_power_bits;       // 0..3
    bool e22_tx_power_permanent;     // C0 vs C2
    bool e22_set_mode;
    e22_mode_t e22_mode;
    uint32_t e22_mode_hold_ms;

    // --- VSENSE ADC test ---
    bool vsense_read_enable;
    adc1_channel_t vsense_chan;      // ex: ADC1_CHANNEL_1
    adc_atten_t vsense_atten;        // ex: ADC_ATTEN_DB_11
    uint32_t vsense_samples;         // ex: 16000
    uint32_t vsense_period_ms;       // ex: 1000
    float adc_full_scale_v;          // ex: 3.3

    // --- Deep sleep ---
    bool go_deep_sleep;
    uint32_t deep_sleep_delay_ms;    // temps pour laisser sortir les logs
} loratube_test_config_t;

loratube_test_config_t loratube_test_config_default(void);

/**
 * Exécute la séquence de test “PCB V3 LORATUBE”.
 * Hypothèse : ctx déjà initialisé via loratube_init().
 */
esp_err_t loratube_run_tests(loratube_ctx_t *ctx, const loratube_test_config_t *t);

esp_err_t loratube_test_fram_scratch(loratube_ctx_t *ctx, uint16_t scratch_addr);

#ifdef __cplusplus
}
#endif
