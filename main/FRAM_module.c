#include "FRAM_module.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

static const char *TAG_FRAM = "FRAM";

typedef struct {
    i2c_port_t port;
    uint8_t    addr;
    TickType_t timeout_ticks;
    bool       inited;
} fram_ctx_t;

static fram_ctx_t s_fram = {0};

// ---------------- CRC8 (poly 0x31, refin/out = true)
// Equivalent to bitwise poly 0x8C in LSB-first loop.
static uint8_t crc8_poly31_reflect_(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    const uint8_t poly = 0x8C; // reflected 0x31

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x01) crc = (uint8_t)((crc >> 1) ^ poly);
            else           crc = (uint8_t)(crc >> 1);
        }
    }
    return crc;
}

static void meta_compute_crc_(fram_meta_t *m)
{
    m->crc8 = crc8_poly31_reflect_((const uint8_t*)m, 15);
}

static bool meta_is_valid_(const fram_meta_t *m)
{
    if (m->magic != FRAM_META_MAGIC) return false;
    const uint8_t c = crc8_poly31_reflect_((const uint8_t*)m, 15);
    return (c == m->crc8);
}

static void log_compute_crc_(fram_daily_log_t *l)
{
    l->crc8 = crc8_poly31_reflect_((const uint8_t*)l, 15);
}

static bool log_is_valid_(const fram_daily_log_t *l)
{
    const uint8_t c = crc8_poly31_reflect_((const uint8_t*)l, 15);
    return (c == l->crc8);
}

// ---------------- Low-level I2C FRAM read/write (16-bit addr) ----------------

static inline TickType_t ms_to_ticks_(uint32_t ms)
{
    if (ms == 0) ms = 1;
    return pdMS_TO_TICKS(ms);
}

static esp_err_t fram_write_chunk_(uint16_t mem_addr, const uint8_t *data, size_t len)
{
    // Buffer: [addr_hi addr_lo data...]
    if (!s_fram.inited) return ESP_ERR_INVALID_STATE;

    // Stack bounded: limit chunk size.
    uint8_t buf[2 + 32];
    if (len > 32) return ESP_ERR_INVALID_SIZE;

    buf[0] = (uint8_t)((mem_addr >> 8) & 0xFF);
    buf[1] = (uint8_t)(mem_addr & 0xFF);
    memcpy(&buf[2], data, len);

    return i2c_master_write_to_device(
        s_fram.port,
        s_fram.addr,
        buf, 2 + len,
        s_fram.timeout_ticks
    );
}

static esp_err_t fram_read_chunk_(uint16_t mem_addr, uint8_t *data, size_t len)
{
    if (!s_fram.inited) return ESP_ERR_INVALID_STATE;

    uint8_t a[2];
    a[0] = (uint8_t)((mem_addr >> 8) & 0xFF);
    a[1] = (uint8_t)(mem_addr & 0xFF);

    return i2c_master_write_read_device(
        s_fram.port,
        s_fram.addr,
        a, sizeof(a),
        data, len,
        s_fram.timeout_ticks
    );
}

esp_err_t fram_write_bytes(uint16_t addr, const void *src, size_t len)
{
    if (!src && len) return ESP_ERR_INVALID_ARG;

    const uint8_t *p = (const uint8_t*)src;
    while (len) {
        const size_t chunk = (len > 32) ? 32 : len;
        esp_err_t r = fram_write_chunk_(addr, p, chunk);
        if (r != ESP_OK) return r;
        addr = (uint16_t)(addr + chunk);
        p += chunk;
        len -= chunk;
    }
    return ESP_OK;
}

esp_err_t fram_read_bytes(uint16_t addr, void *dst, size_t len)
{
    if (!dst && len) return ESP_ERR_INVALID_ARG;

    uint8_t *p = (uint8_t*)dst;
    while (len) {
        const size_t chunk = (len > 32) ? 32 : len;
        esp_err_t r = fram_read_chunk_(addr, p, chunk);
        if (r != ESP_OK) return r;
        addr = (uint16_t)(addr + chunk);
        p += chunk;
        len -= chunk;
    }
    return ESP_OK;
}

// ---------------- Memory map helpers ----------------

static inline uint16_t meta_a_addr_(void) { return 0x0000u; }
static inline uint16_t meta_b_addr_(void) { return (uint16_t)(FRAM_SIZE_BYTES - sizeof(fram_meta_t)); }
static inline uint16_t logs_base_addr_(void) { return (uint16_t)(meta_a_addr_() + sizeof(fram_meta_t)); }
static inline uint16_t logs_end_addr_exclusive_(void) { return meta_b_addr_(); }

uint16_t fram_log_capacity(void)
{
    const uint16_t bytes = (uint16_t)(logs_end_addr_exclusive_() - logs_base_addr_());
    return (uint16_t)(bytes / (uint16_t)sizeof(fram_daily_log_t));
}

static inline uint16_t log_addr_from_index_(uint16_t idx)
{
    return (uint16_t)(logs_base_addr_() + (uint16_t)(idx * (uint16_t)sizeof(fram_daily_log_t)));
}

// ---------------- Public API ----------------

esp_err_t fram_init(i2c_port_t port, uint8_t i2c_addr, uint32_t timeout_ms)
{
    s_fram.port = port;
    s_fram.addr = i2c_addr;
    s_fram.timeout_ticks = ms_to_ticks_(timeout_ms ? timeout_ms : FRAM_TIMEOUT_MS_DEFAULT);
    s_fram.inited = true;

    // Smoke: try reading meta A
    fram_meta_t tmp = {0};
    esp_err_t r = fram_read_bytes(meta_a_addr_(), &tmp, sizeof(tmp));
    if (r != ESP_OK) {
        ESP_LOGE(TAG_FRAM, "FRAM NACK/read failed @0x%04X: %s", meta_a_addr_(), esp_err_to_name(r));
        return r;
    }

    ESP_LOGI(TAG_FRAM, "FRAM init OK (addr=0x%02X, cap=%u logs)", s_fram.addr, (unsigned)fram_log_capacity());
    return ESP_OK;
}

static esp_err_t meta_read_at_(uint16_t addr, fram_meta_t *out)
{
    return fram_read_bytes(addr, out, sizeof(*out));
}

static esp_err_t meta_write_verified_(uint16_t addr, const fram_meta_t *m)
{
    // write + readback compare
    esp_err_t r = fram_write_bytes(addr, m, sizeof(*m));
    if (r != ESP_OK) return r;

    fram_meta_t rb;
    r = fram_read_bytes(addr, &rb, sizeof(rb));
    if (r != ESP_OK) return r;

    if (memcmp(&rb, m, sizeof(*m)) != 0) return ESP_ERR_INVALID_RESPONSE;
    if (!meta_is_valid_(&rb)) return ESP_ERR_INVALID_CRC;

    return ESP_OK;
}

esp_err_t fram_meta_store(const fram_meta_t *meta)
{
    if (!meta) return ESP_ERR_INVALID_ARG;
    if (!s_fram.inited) return ESP_ERR_INVALID_STATE;

    fram_meta_t m = *meta;
    meta_compute_crc_(&m);

    for (int attempt = 0; attempt < (int)FRAM_RETRY_MAX; attempt++) {
        esp_err_t r = meta_write_verified_(meta_a_addr_(), &m);
        if (r != ESP_OK) {
            ESP_LOGW(TAG_FRAM, "Meta write/verify A failed (try %d/%u): %s",
                     attempt + 1, (unsigned)FRAM_RETRY_MAX, esp_err_to_name(r));
            continue;
        }

        r = meta_write_verified_(meta_b_addr_(), &m);
        if (r != ESP_OK) {
            ESP_LOGW(TAG_FRAM, "Meta write/verify B failed (try %d/%u): %s",
                     attempt + 1, (unsigned)FRAM_RETRY_MAX, esp_err_to_name(r));
            continue;
        }

        return ESP_OK;
    }

    return ESP_FAIL;
}

esp_err_t fram_meta_load(fram_meta_t *out_meta, fram_meta_source_t *src_used)
{
    if (!out_meta) return ESP_ERR_INVALID_ARG;
    if (!s_fram.inited) return ESP_ERR_INVALID_STATE;

    fram_meta_t a = {0}, b = {0};
    esp_err_t ra = meta_read_at_(meta_a_addr_(), &a);
    esp_err_t rb = meta_read_at_(meta_b_addr_(), &b);

    const bool a_ok = (ra == ESP_OK) && meta_is_valid_(&a);
    const bool b_ok = (rb == ESP_OK) && meta_is_valid_(&b);

    if (a_ok && b_ok) {
        *out_meta = a;
        if (src_used) *src_used = FRAM_META_SRC_A;
        return ESP_OK;
    }

    if (a_ok && !b_ok) {
        *out_meta = a;
        out_meta->flags |= FRAM_FLAG_CORRUPTED_FRAM;
        meta_compute_crc_(out_meta);
        if (src_used) *src_used = FRAM_META_SRC_A;
        (void)fram_meta_store(out_meta); // best-effort repair
        return ESP_OK;
    }

    if (!a_ok && b_ok) {
        *out_meta = b;
        out_meta->flags |= FRAM_FLAG_CORRUPTED_FRAM;
        meta_compute_crc_(out_meta);
        if (src_used) *src_used = FRAM_META_SRC_B;
        (void)fram_meta_store(out_meta); // best-effort repair
        return ESP_OK;
    }

    // Neither valid -> reset
    memset(out_meta, 0, sizeof(*out_meta));
    out_meta->magic = FRAM_META_MAGIC;
    out_meta->index = 0;
    out_meta->wake_rx = 0;
    out_meta->true_rx = 0;
    out_meta->tx = 0;
    out_meta->noise_min = 0xFF; // "unset"
    out_meta->noise_max = 0x00;
    out_meta->flags = FRAM_FLAG_CORRUPTED_FRAM;
    meta_compute_crc_(out_meta);

    if (src_used) *src_used = FRAM_META_SRC_A;
    return fram_meta_store(out_meta);
}

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
)
{
    if (!out) return;

    memset(out, 0, sizeof(*out));

    out->timestamp_unix = timestamp_unix;
    out->midnight_temp_c = midnight_temp_c;
    out->midnight_voltage_raw = midnight_voltage_raw;
    out->noon_temp_c = noon_temp_c;
    out->noon_voltage_raw = noon_voltage_raw;

    out->wake_rx = wake_rx;

    // ratio byte10: true_rx per wake
    if (wake_rx == 0) out->ratio_rxtrue_per_wake_q8 = 0;
    else {
        uint32_t v = (255u * (uint32_t)true_rx + (uint32_t)(wake_rx / 2u)) / (uint32_t)wake_rx;
        if (v > 255u) v = 255u;
        out->ratio_rxtrue_per_wake_q8 = (uint8_t)v;
    }

    // ratio byte11: tx per true_rx
    if (true_rx == 0) out->ratio_tx_per_rxtrue_q8 = 0;
    else {
        uint32_t v = (255u * (uint32_t)tx + (uint32_t)(true_rx / 2u)) / (uint32_t)true_rx;
        if (v > 255u) v = 255u;
        out->ratio_tx_per_rxtrue_q8 = (uint8_t)v;
    }

    out->flags = flags;
    out->noise_min_raw = noise_min_raw;
    out->noise_max_raw = noise_max_raw;

    log_compute_crc_(out);
}

static esp_err_t log_write_verified_(uint16_t addr, const fram_daily_log_t *l)
{
    esp_err_t r = fram_write_bytes(addr, l, sizeof(*l));
    if (r != ESP_OK) return r;

    fram_daily_log_t rb;
    r = fram_read_bytes(addr, &rb, sizeof(rb));
    if (r != ESP_OK) return r;

    if (memcmp(&rb, l, sizeof(*l)) != 0) return ESP_ERR_INVALID_RESPONSE;
    if (!log_is_valid_(&rb)) return ESP_ERR_INVALID_CRC;

    return ESP_OK;
}

esp_err_t fram_log_append(const fram_daily_log_t *log, fram_meta_t *inout_meta)
{
    if (!log || !inout_meta) return ESP_ERR_INVALID_ARG;
    if (!s_fram.inited) return ESP_ERR_INVALID_STATE;

    fram_daily_log_t l = *log;
    log_compute_crc_(&l); // ensure CRC is consistent

    const uint16_t cap = fram_log_capacity();
    if (cap == 0) return ESP_ERR_INVALID_SIZE;

    uint16_t idx = inout_meta->index;
    if (idx >= cap) idx = 0;

    const uint16_t addr = log_addr_from_index_(idx);

    for (int attempt = 0; attempt < (int)FRAM_RETRY_MAX; attempt++) {
        esp_err_t r = log_write_verified_(addr, &l);
        if (r == ESP_OK) {
            // advance index + persist meta
            uint16_t next = (uint16_t)(idx + 1u);
            if (next >= cap) next = 0;
            inout_meta->index = next;

            return fram_meta_store(inout_meta);
        }

        ESP_LOGW(TAG_FRAM, "Log write/verify failed idx=%u addr=0x%04X (try %d/%u): %s",
                 (unsigned)idx, (unsigned)addr, attempt + 1, (unsigned)FRAM_RETRY_MAX, esp_err_to_name(r));
    }

    return ESP_FAIL;
}

esp_err_t fram_log_read(uint16_t index, fram_daily_log_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    if (!s_fram.inited) return ESP_ERR_INVALID_STATE;

    const uint16_t cap = fram_log_capacity();
    if (cap == 0) return ESP_ERR_INVALID_SIZE;
    if (index >= cap) return ESP_ERR_INVALID_ARG;

    const uint16_t addr = log_addr_from_index_(index);
    esp_err_t r = fram_read_bytes(addr, out, sizeof(*out));
    if (r != ESP_OK) return r;

    if (!log_is_valid_(out)) return ESP_ERR_INVALID_CRC;
    return ESP_OK;
}
