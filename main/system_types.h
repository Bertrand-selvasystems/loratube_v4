#pragma once
#include <stdint.h>

// rassemble toutes les structures inter modules

typedef struct {
    int16_t c10;      // Température en °C * 10
    uint32_t raw;     // optionnel : brute TSENS si tu veux
    float temp_c;
} c3_temp_msg_t;

typedef struct {
    uint16_t mv;      // Tension batterie en mV
    uint32_t k_mean;  // optionnel : ADC mean
    uint32_t k_span;  // optionnel : ADC span
    float vbat_v;
} c3_vbat_msg_t;
