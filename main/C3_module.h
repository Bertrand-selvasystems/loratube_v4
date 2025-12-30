#pragma once

#include "esp_check.h"
#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "C3_types.h"

// ADC legacy types (si tu gardes adc1_config_* / adc1_get_raw)
#include "driver/adc.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================
//  Board pins : VSENSE
// =============================
// Assure visibilité dans tous les .c qui incluent C3_module.h
#ifndef VSENSE_GPIO
#define VSENSE_GPIO (GPIO_NUM_1)
#endif

#ifndef VSENSE_CHAN
#define VSENSE_CHAN   ADC1_CHANNEL_1
#endif

// =============================
//  TEMP SENSOR (ESP32-C3 TSENS)
// =============================

typedef struct {
    int dac_offset;
    uint8_t read_retries;
} diag_tsens_cfg_t;

esp_err_t diag_tsens_start(const diag_tsens_cfg_t *cfg);
esp_err_t diag_tsens_stop(void);
esp_err_t diag_tsens_read(float *out_celsius, uint32_t *out_raw);
void diag_tsens_log_periodic(const char *tag, uint32_t samples, uint32_t period_ms);

// =============================
//  VSENSE / PACK VOLTAGE (ADC)
// =============================

typedef struct {
    uint16_t samples_per_read;
    uint16_t intersample_delay_us;
    adc_atten_t atten;          // ex: ADC_ATTEN_DB_12 (DB_11 deprecated)
    adc_bits_width_t width;     // ex: ADC_WIDTH_BIT_12
} diag_vsense_cfg_t;

esp_err_t diag_vsense_start(const diag_vsense_cfg_t *cfg);
esp_err_t diag_vsense_stop(void);
esp_err_t diag_vsense_read(double *out_vpack, uint32_t *out_k_mean, uint32_t *out_k_span);
void diag_vsense_log_periodic(const char *tag, uint32_t samples, uint32_t period_ms);
double diag_vsense_adc_to_voltage(double K);


#ifdef __cplusplus
}
#endif
