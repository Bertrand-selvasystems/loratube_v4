#include "pcf8523_module.h"

#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"   // pdMS_TO_TICKS
#include "esp_log.h"

static const char *TAG = "PCF8523";

static inline TickType_t ms_to_ticks_(uint32_t ms)
{
    if (ms == 0) ms = 1;
    return pdMS_TO_TICKS(ms);
}

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
    return ESP_OK;
}

esp_err_t pcf8523_read_reg(pcf8523_t *dev, uint8_t reg, uint8_t *val)
{
    return reg_read_(dev, reg, val);
}

esp_err_t pcf8523_write_reg(pcf8523_t *dev, uint8_t reg, uint8_t val)
{
    return reg_write_(dev, reg, val);
}

esp_err_t pcf8523_debug_init_irq_timer(pcf8523_t *dev, bool log_readback)
{
    if (!dev || !dev->initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t ret;
    uint8_t v = 0;

    if (log_readback) {
        ret = reg_read_(dev, PCF8523_REG_TMR_CLKOUT_CTRL, &v);
        if (ret != ESP_OK) return ret;
        ESP_LOGW(TAG, "CLKOUT(0x0F) initial = 0x%02X", v);

        ret = reg_read_(dev, PCF8523_REG_CONTROL_2, &v);
        if (ret != ESP_OK) return ret;
        ESP_LOGW(TAG, "CTRL2 (0x01) initial = 0x%02X", v);
    }

    // 1) Désactive CLKOUT (simple)
    ret = reg_write_(dev, PCF8523_REG_TMR_CLKOUT_CTRL, 0x00);
    if (ret != ESP_OK) return ret;

    // 2) Control_2 : TI_TP=1 (pulse), TIE=1, flags cleared
    uint8_t ctrl2 = PCF8523_CTRL2_TI_TP | PCF8523_CTRL2_TIE;
    ret = reg_write_(dev, PCF8523_REG_CONTROL_2, ctrl2);
    if (ret != ESP_OK) return ret;

    if (log_readback) {
        ret = reg_read_(dev, PCF8523_REG_TMR_CLKOUT_CTRL, &v);
        if (ret != ESP_OK) return ret;
        ESP_LOGI(TAG, "CLKOUT(0x0F) after = 0x%02X (exp 0x00)", v);

        ret = reg_read_(dev, PCF8523_REG_CONTROL_2, &v);
        if (ret != ESP_OK) return ret;
        ESP_LOGI(TAG, "CTRL2 (0x01) after  = 0x%02X (exp TI_TP=1,TIE=1)", v);
    }

    return ESP_OK;
}

esp_err_t pcf8523_make_sqw_hi_z(pcf8523_t *dev, bool log_readback)
{
    if (!dev || !dev->initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t ret;
    uint8_t v = 0;

    // 0x38 : COF=OFF, timers off (ton choix)
    ret = reg_write_(dev, PCF8523_REG_TMR_CLKOUT_CTRL, 0x38);
    if (ret != ESP_OK) return ret;

    // IRQ off + clear flags
    ret = reg_write_(dev, PCF8523_REG_CONTROL_2, 0x00);
    if (ret != ESP_OK) return ret;

    if (log_readback) {
        ret = reg_read_(dev, PCF8523_REG_TMR_CLKOUT_CTRL, &v);
        if (ret != ESP_OK) return ret;
        ESP_LOGI(TAG, "0x0F=Tmr_CLKOUT_ctrl -> 0x%02X (exp 0x38)", v);

        ret = reg_read_(dev, PCF8523_REG_CONTROL_2, &v);
        if (ret != ESP_OK) return ret;
        ESP_LOGI(TAG, "0x01=Control_2      -> 0x%02X (exp 0x00)", v);
    }

    return ESP_OK;
}
