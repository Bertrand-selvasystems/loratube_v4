#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"
#include "driver/i2c.h"

// --- FRAM device: MB85RC256V, 32 kB I2C FRAM ---
#ifndef FRAM_I2C_ADDR
#define FRAM_I2C_ADDR              (0x50)
#endif

#define FRAM_SIZE_BYTES            (32768u)   // 32 kB
#define FRAM_TIMEOUT_MS_DEFAULT    (50u)
#define FRAM_RETRY_MAX             (3u)

// Certains IDF n'ont pas ESP_ERR_INVALID_CRC exposé.
// On retombe sur INVALID_RESPONSE pour garder une sémantique claire.
#ifndef ESP_ERR_INVALID_CRC
#define ESP_ERR_INVALID_CRC        ESP_ERR_INVALID_RESPONSE
#endif

// --- Memory map ---
#define FRAM_META_MAGIC            (0x4C544D45u) // 'LTME'

typedef enum {
    FRAM_META_SRC_A = 0,
    FRAM_META_SRC_B = 1,
} fram_meta_source_t;

// Flags byte
typedef enum {
    FRAM_FLAG_TEMP_EXT       = (1u << 0),
    FRAM_FLAG_E22_RESET      = (1u << 1),
    FRAM_FLAG_C3_MISSED_TIME = (1u << 2),
    FRAM_FLAG_LOW_VOLT       = (1u << 3),
    FRAM_FLAG_WAKE_UART      = (1u << 4),
    FRAM_FLAG_CORRUPTED_FRAM = (1u << 5),
    FRAM_FLAG_C3_RESET_REQ   = (1u << 6),
    FRAM_FLAG_C3_BROWNOUT    = (1u << 7),
} fram_flags_t;

// --- Daily log (16 bytes, little-endian, CRC8 on first 15 bytes) ---
typedef struct __attribute__((packed)) {
    uint32_t timestamp_unix;              // 0..3
    int8_t   midnight_temp_c;             // 4
    uint8_t  midnight_voltage_raw;        // 5
    int8_t   noon_temp_c;                 // 6
    uint8_t  noon_voltage_raw;            // 7
    uint16_t wake_rx;                     // 8..9
    uint8_t  ratio_rxtrue_per_wake_q8;    // 10
    uint8_t  ratio_tx_per_rxtrue_q8;      // 11
    uint8_t  flags;                       // 12
    uint8_t  noise_min_raw;               // 13
    uint8_t  noise_max_raw;               // 14
    uint8_t  crc8;                        // 15
} fram_daily_log_t;

// --- Meta slot (16 bytes, CRC8 on first 15 bytes) ---
typedef struct __attribute__((packed)) {
    uint32_t magic;       // FRAM_META_MAGIC
    uint16_t index;       // next write index [0..capacity-1]
    uint16_t wake_rx;     // daily counter
    uint16_t true_rx;     // daily counter
    uint16_t tx;          // daily counter
    uint8_t  noise_min;   // daily min (raw)
    uint8_t  noise_max;   // daily max (raw)
    uint8_t  flags;       // accumulated flags (OR)
    uint8_t  crc8;        // CRC8 of first 15 bytes
} fram_meta_t;

// --- Public API ---
esp_err_t fram_init(i2c_port_t port, uint8_t i2c_addr, uint32_t timeout_ms);

esp_err_t fram_meta_load(fram_meta_t *out_meta, fram_meta_source_t *src_used);
esp_err_t fram_meta_store(const fram_meta_t *meta);

esp_err_t fram_log_append(const fram_daily_log_t *log, fram_meta_t *inout_meta);

void fram_log_build(
    fram_daily_log_t *out,
    uint32_t timestamp_unix,
    int8_t midnight_temp_c,
    uint8_t midnight_voltage_raw,
    int8_t noon_temp_c,
    uint8_t noon_voltage_raw,
    uint16_t wake_rx,
    uint16_t true_rx,
    uint16_t tx,
    uint8_t flags,
    uint8_t noise_min_raw,
    uint8_t noise_max_raw
);

esp_err_t fram_log_read(uint16_t index, fram_daily_log_t *out);
uint16_t fram_log_capacity(void);

// --- Raw access ---
esp_err_t fram_read_bytes(uint16_t addr, void *dst, size_t len);
esp_err_t fram_write_bytes(uint16_t addr, const void *src, size_t len);
