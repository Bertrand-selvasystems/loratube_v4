#include "PCF8523_module.h"

#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "esp_log.h"

static const char *TAG = "PCF8523";

// ---------- timeouts ----------
static inline TickType_t ms_to_ticks_(uint32_t ms)
{
    if (ms == 0) ms = 1;
    TickType_t t = pdMS_TO_TICKS(ms);
    return (t == 0) ? 1 : t;
}

// ---------- low-level I2C ----------
static esp_err_t reg_read_(pcf8523_t *dev, uint8_t reg, uint8_t *val)
{
    if (!dev || !dev->initialized || !val) return ESP_ERR_INVALID_STATE;

    const TickType_t to = ms_to_ticks_(dev->timeout_ms);

    esp_err_t ret = i2c_master_write_read_device(
        dev->port,
        dev->addr_7bit,
        &reg, 1,
        val, 1,
        to
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read reg 0x%02X failed: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t reg_write_(pcf8523_t *dev, uint8_t reg, uint8_t val)
{
    if (!dev || !dev->initialized) return ESP_ERR_INVALID_STATE;

    const TickType_t to = ms_to_ticks_(dev->timeout_ms);

    uint8_t buf[2] = { reg, val };
    esp_err_t ret = i2c_master_write_to_device(
        dev->port,
        dev->addr_7bit,
        buf, sizeof(buf),
        to
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write reg 0x%02X=0x%02X failed: %s", reg, val, esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t burst_read_(pcf8523_t *dev, uint8_t start_reg, uint8_t *buf, size_t len)
{
    if (!dev || !dev->initialized || !buf || len == 0) return ESP_ERR_INVALID_ARG;

    const TickType_t to = ms_to_ticks_(dev->timeout_ms);

    esp_err_t ret = i2c_master_write_read_device(
        dev->port,
        dev->addr_7bit,
        &start_reg, 1,
        buf, len,
        to
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Burst read reg 0x%02X len=%u failed: %s",
                 start_reg, (unsigned)len, esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t burst_write_(pcf8523_t *dev, uint8_t start_reg, const uint8_t *buf, size_t len)
{
    if (!dev || !dev->initialized || !buf || len == 0) return ESP_ERR_INVALID_ARG;

    const TickType_t to = ms_to_ticks_(dev->timeout_ms);

    // stack buffer for small writes; otherwise reject (keep module simple & deterministic)
    if (len > 32) return ESP_ERR_INVALID_SIZE;

    uint8_t tmp[1 + 32];
    tmp[0] = start_reg;
    memcpy(&tmp[1], buf, len);

    esp_err_t ret = i2c_master_write_to_device(dev->port, dev->addr_7bit, tmp, 1 + len, to);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Burst write reg 0x%02X len=%u failed: %s",
                 start_reg, (unsigned)len, esp_err_to_name(ret));
    }
    return ret;
}

// ---------- BCD helpers ----------
static inline uint8_t bcd2bin_(uint8_t v) { return (uint8_t)((v >> 4) * 10 + (v & 0x0F)); }
static inline uint8_t bin2bcd_(uint8_t v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }

// mask helpers per register (PCF8523 uses MSBs for flags in some time regs)
static inline uint8_t sec_mask_(uint8_t v)   { return (uint8_t)(v & 0x7F); } // OS flag may be bit7
static inline uint8_t min_mask_(uint8_t v)   { return (uint8_t)(v & 0x7F); }
static inline uint8_t hour_mask_(uint8_t v)  { return (uint8_t)(v & 0x3F); }
static inline uint8_t day_mask_(uint8_t v)   { return (uint8_t)(v & 0x3F); }
static inline uint8_t wday_mask_(uint8_t v)  { return (uint8_t)(v & 0x07); }
static inline uint8_t month_mask_(uint8_t v) { return (uint8_t)(v & 0x1F); }
static inline uint8_t year_mask_(uint8_t v)  { return v; }

// ---------- public API ----------
esp_err_t pcf8523_init(pcf8523_t *dev, i2c_port_t port, uint8_t addr_7bit, uint32_t timeout_ms)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    memset(dev, 0, sizeof(*dev));
    dev->port = port;
    dev->addr_7bit = addr_7bit;
    dev->timeout_ms = timeout_ms ? timeout_ms : 50;
    dev->initialized = true;

    ESP_LOGI(TAG, "Init OK (port=%d addr=0x%02X timeout=%" PRIu32 "ms)",
             (int)port, (unsigned)addr_7bit, dev->timeout_ms);

    // Optional sanity ping: read Control_1
    uint8_t c1 = 0;
    esp_err_t ret = reg_read_(dev, PCF8523_REG_CONTROL_1, &c1);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

esp_err_t pcf8523_read_reg(pcf8523_t *dev, uint8_t reg, uint8_t *val) { return reg_read_(dev, reg, val); }
esp_err_t pcf8523_write_reg(pcf8523_t *dev, uint8_t reg, uint8_t val) { return reg_write_(dev, reg, val); }
esp_err_t pcf8523_read(pcf8523_t *dev, uint8_t start_reg, uint8_t *buf, size_t len) { return burst_read_(dev, start_reg, buf, len); }
esp_err_t pcf8523_write(pcf8523_t *dev, uint8_t start_reg, const uint8_t *buf, size_t len) { return burst_write_(dev, start_reg, buf, len); }

// ---------- datetime ----------
esp_err_t pcf8523_get_datetime(pcf8523_t *dev, pcf8523_datetime_t *dt)
{
    if (!dev || !dev->initialized || !dt) return ESP_ERR_INVALID_ARG;

    uint8_t r[7] = {0};
    esp_err_t ret = burst_read_(dev, PCF8523_REG_SECONDS, r, sizeof(r));
    if (ret != ESP_OK) return ret;

    dt->sec   = bcd2bin_(sec_mask_(r[0]));
    dt->min   = bcd2bin_(min_mask_(r[1]));
    dt->hour  = bcd2bin_(hour_mask_(r[2]));
    dt->day   = bcd2bin_(day_mask_(r[3]));
    dt->wday  = bcd2bin_(wday_mask_(r[4]));      // typically plain 0..6
    dt->month = bcd2bin_(month_mask_(r[5]));
    dt->year  = (uint16_t)(2000u + bcd2bin_(year_mask_(r[6])));

    return ESP_OK;
}

esp_err_t pcf8523_set_datetime(pcf8523_t *dev, const pcf8523_datetime_t *dt)
{
    if (!dev || !dev->initialized || !dt) return ESP_ERR_INVALID_ARG;

    if (dt->month < 1 || dt->month > 12) return ESP_ERR_INVALID_ARG;
    if (dt->day < 1 || dt->day > 31) return ESP_ERR_INVALID_ARG;
    if (dt->hour > 23 || dt->min > 59 || dt->sec > 59) return ESP_ERR_INVALID_ARG;
    if (dt->year < 2000 || dt->year > 2099) return ESP_ERR_INVALID_ARG;

    uint8_t r[7];
    r[0] = bin2bcd_(dt->sec)  & 0x7F;
    r[1] = bin2bcd_(dt->min)  & 0x7F;
    r[2] = bin2bcd_(dt->hour) & 0x3F;
    r[3] = bin2bcd_(dt->day)  & 0x3F;
    r[4] = (uint8_t)(dt->wday & 0x07);
    r[5] = bin2bcd_(dt->month) & 0x1F;
    r[6] = bin2bcd_((uint8_t)(dt->year - 2000));

    return burst_write_(dev, PCF8523_REG_SECONDS, r, sizeof(r));
}

// ---------- CLKOUT ----------
esp_err_t pcf8523_clkout_set(pcf8523_t *dev, uint8_t cof_value_0_7, bool log_readback)
{
    if (!dev || !dev->initialized) return ESP_ERR_INVALID_STATE;
    if (cof_value_0_7 > 7) return ESP_ERR_INVALID_ARG;

    uint8_t v;
    esp_err_t ret = reg_read_(dev, PCF8523_REG_TMR_CLKOUT, &v);
    if (ret != ESP_OK) return ret;

    if (log_readback) {
        ESP_LOGI(TAG, "0x0F before = 0x%02X", v);
    }

    // preserve timers config, only change COF field
    v &= (uint8_t)~PCF8523_TMRCLKOUT_COF_MASK;
    v |= PCF8523_TMRCLKOUT_COF(cof_value_0_7);

    ret = reg_write_(dev, PCF8523_REG_TMR_CLKOUT, v);
    if (ret != ESP_OK) return ret;

    if (log_readback) {
        uint8_t rd = 0;
        ret = reg_read_(dev, PCF8523_REG_TMR_CLKOUT, &rd);
        if (ret != ESP_OK) return ret;
        ESP_LOGI(TAG, "0x0F after  = 0x%02X (COF=%u)", rd, (unsigned)cof_value_0_7);
    }

    return ESP_OK;
}

esp_err_t pcf8523_clkout_disable(pcf8523_t *dev, bool log_readback)
{
    if (!dev || !dev->initialized) return ESP_ERR_INVALID_STATE;

    // Hard-safe baseline used widely: COF=disable + timers off
    esp_err_t ret = reg_write_(dev, PCF8523_REG_TMR_CLKOUT, PCF8523_TMRCLKOUT_ALL_OFF);
    if (ret != ESP_OK) return ret;

    // Disable all Control_2 IRQ enables (keep flags clear handling separate)
    ret = reg_write_(dev, PCF8523_REG_CONTROL_2, 0x00);
    if (ret != ESP_OK) return ret;

    if (log_readback) {
        uint8_t v = 0;
        ret = reg_read_(dev, PCF8523_REG_TMR_CLKOUT, &v);
        if (ret != ESP_OK) return ret;
        ESP_LOGI(TAG, "0x0F=Tmr_CLKOUT_ctrl -> 0x%02X (exp 0x38)", v);

        ret = reg_read_(dev, PCF8523_REG_CONTROL_2, &v);
        if (ret != ESP_OK) return ret;
        ESP_LOGI(TAG, "0x01=Control_2      -> 0x%02X (exp 0x00)", v);
    }

    return ESP_OK;
}

// ---------- Control_2 flags clearing (safe, does not clobber enables) ----------
esp_err_t pcf8523_clear_flags(pcf8523_t *dev, uint8_t flags_to_clear, bool log_readback)
{
    if (!dev || !dev->initialized) return ESP_ERR_INVALID_STATE;

    uint8_t c2 = 0;
    esp_err_t ret = reg_read_(dev, PCF8523_REG_CONTROL_2, &c2);
    if (ret != ESP_OK) return ret;

    if (log_readback) {
        ESP_LOGI(TAG, "CTRL2 before = 0x%02X", c2);
    }

    // Preserve enables (bits2..0)
    const uint8_t enables = (uint8_t)(c2 & PCF8523_CTRL2_IE_MASK);

    // Flags clearing on PCF8523 is not a naive W1C; we do the conservative approach:
    // write back enables + "keep flags not requested". For flags requested, we force them low.
    // This mirrors the datasheet example style.
    uint8_t flags_keep = (uint8_t)(PCF8523_CTRL2_FLAGS_MASK & (uint8_t)~flags_to_clear);
    uint8_t w = (uint8_t)(enables | flags_keep);

    ret = reg_write_(dev, PCF8523_REG_CONTROL_2, w);
    if (ret != ESP_OK) return ret;

    if (log_readback) {
        uint8_t rd = 0;
        ret = reg_read_(dev, PCF8523_REG_CONTROL_2, &rd);
        if (ret != ESP_OK) return ret;
        ESP_LOGI(TAG, "CTRL2 after  = 0x%02X (wrote 0x%02X)", rd, w);
    }

    return ESP_OK;
}

// ---------- legacy "debug init irq timer" but now correct ----------
esp_err_t pcf8523_debug_init_irq_timer(pcf8523_t *dev, bool enable_timer_b_irq, bool log_readback)
{
    if (!dev || !dev->initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t ret;
    uint8_t v = 0;

    if (log_readback) {
        ret = reg_read_(dev, PCF8523_REG_TMR_CLKOUT, &v);
        if (ret != ESP_OK) return ret;
        ESP_LOGW(TAG, "0x0F initial = 0x%02X", v);

        ret = reg_read_(dev, PCF8523_REG_CONTROL_2, &v);
        if (ret != ESP_OK) return ret;
        ESP_LOGW(TAG, "0x01 initial = 0x%02X", v);
    }

    // Baseline: CLKOUT disabled + timers off (0x38) (common safe value)
    ret = reg_write_(dev, PCF8523_REG_TMR_CLKOUT, PCF8523_TMRCLKOUT_ALL_OFF);
    if (ret != ESP_OK) return ret;

    // Clear flags without clobbering enables, then optionally enable Timer B IRQ
    ret = pcf8523_clear_flags(dev, PCF8523_CTRL2_FLAGS_MASK, false);
    if (ret != ESP_OK) return ret;

    uint8_t c2 = 0;
    ret = reg_read_(dev, PCF8523_REG_CONTROL_2, &c2);
    if (ret != ESP_OK) return ret;

    // Keep flags in cleared state; enable CTBIE if asked
    c2 &= (uint8_t)~PCF8523_CTRL2_IE_MASK;
    if (enable_timer_b_irq) c2 |= PCF8523_CTRL2_CTBIE;

    ret = reg_write_(dev, PCF8523_REG_CONTROL_2, c2);
    if (ret != ESP_OK) return ret;

    if (log_readback) {
        ret = reg_read_(dev, PCF8523_REG_TMR_CLKOUT, &v);
        if (ret != ESP_OK) return ret;
        ESP_LOGI(TAG, "0x0F after   = 0x%02X (exp 0x38)", v);

        ret = reg_read_(dev, PCF8523_REG_CONTROL_2, &v);
        if (ret != ESP_OK) return ret;
        ESP_LOGI(TAG, "0x01 after   = 0x%02X (CTBIE=%d)", v, enable_timer_b_irq ? 1 : 0);
    }

    return ESP_OK;
}
