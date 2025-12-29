#include "PCA9536_module.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static const char *TAG = "PCA9536";

static TickType_t ms_to_ticks_(uint32_t ms)
{
    if (ms == 0) ms = 1;
    return pdMS_TO_TICKS(ms);
}



pca9536_config_t pca9536_config_default(i2c_port_t port)
{
    pca9536_config_t c = {
        .port = port,
        .addr = PCA9536_ADDR,
        .timeout_ms = 50,
    };
    return c;
}

esp_err_t pca9536_init(pca9536_handle_t *h, const pca9536_config_t *cfg)
{
    if (!h || !cfg) return ESP_ERR_INVALID_ARG;

    memset(h, 0, sizeof(*h));
    h->cfg = *cfg;


    h->mutex = xSemaphoreCreateMutex();
    if (!h->mutex) return ESP_ERR_NO_MEM;

    // Polarity inversion off
    esp_err_t ret = pca9536_write_reg(h, PCA9536_REG_POLINV, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "POLINV write failed: %s", esp_err_to_name(ret));
        pca9536_deinit(h);
        return ret;
    }

    // GP0..GP3 outputs
    ret = pca9536_write_reg(h, PCA9536_REG_CONFIG, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CONFIG write failed: %s", esp_err_to_name(ret));
        pca9536_deinit(h);
        return ret;
    }

    // OUTPUT = 0 (state known)
    ret = pca9536_write_reg(h, PCA9536_REG_OUTPUT, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OUTPUT write failed: %s", esp_err_to_name(ret));
        pca9536_deinit(h);
        return ret;
    }

    h->out_cache = 0x00;
    h->out_valid = true;
    h->initialized = true;
    ESP_LOGI(TAG, "init OK (port=%d addr=0x%02X timeout=%" PRIu32 "ms)",
             (int)h->cfg.port, (unsigned)h->cfg.addr, h->cfg.timeout_ms);
    return ESP_OK;
}

esp_err_t pca9536_deinit(pca9536_handle_t *h)
{
    if (!h) return ESP_ERR_INVALID_ARG;

    if (h->mutex) {
        vSemaphoreDelete(h->mutex);
        h->mutex = NULL;
    }
    h->out_valid = false;
    h->initialized = false;
    return ESP_OK;
}

esp_err_t pca9536_read_reg(pca9536_handle_t *h, uint8_t reg, uint8_t *val)
{
    if (!h || !val) return ESP_ERR_INVALID_ARG;
    if (!h->mutex) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(h->mutex, ms_to_ticks_(h->cfg.timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = i2c_master_write_read_device(
        h->cfg.port, h->cfg.addr,
        &reg, 1,
        val, 1,
        ms_to_ticks_(h->cfg.timeout_ms)
    );

    if (ret == ESP_OK && reg == PCA9536_REG_OUTPUT) {
        h->out_cache = (uint8_t)(*val & 0x0F);
        h->out_valid = true;
    }

    xSemaphoreGive(h->mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "read reg 0x%02X failed: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}


esp_err_t pca9536_write_reg(pca9536_handle_t *h, uint8_t reg, uint8_t val)
{
    if (!h) return ESP_ERR_INVALID_ARG;
    if (!h->mutex) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(h->mutex, ms_to_ticks_(h->cfg.timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    uint8_t buf[2] = { reg, val };
    esp_err_t ret = i2c_master_write_to_device(
        h->cfg.port, h->cfg.addr,
        buf, sizeof(buf),
        ms_to_ticks_(h->cfg.timeout_ms)
    );

if (ret == ESP_OK) {
    if (reg == PCA9536_REG_OUTPUT) {
        h->out_cache = (uint8_t)(val & 0x0F);
        h->out_valid = true;
    } else if (reg == PCA9536_REG_CONFIG) {
        h->out_valid = false;
    }
}

    xSemaphoreGive(h->mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "write reg 0x%02X failed: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}


esp_err_t pca9536_set_output_bit(pca9536_handle_t *h, uint8_t bit, bool level)
{
    if (!h) return ESP_ERR_INVALID_ARG;
    if (bit > 3) return ESP_ERR_INVALID_ARG;
    if (!h->mutex) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(h->mutex, ms_to_ticks_(h->cfg.timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // si cache pas valide, on le resynchronise une fois
    if (!h->out_valid) {
        uint8_t reg = PCA9536_REG_OUTPUT;
        uint8_t cur = 0;
        esp_err_t r = i2c_master_write_read_device(
            h->cfg.port, h->cfg.addr,
            &reg, 1,
            &cur, 1,
            ms_to_ticks_(h->cfg.timeout_ms)
        );
        if (r != ESP_OK) {
            xSemaphoreGive(h->mutex);
            ESP_LOGE(TAG, "Read OUTPUT failed: %s", esp_err_to_name(r));
            return r;
        }
        h->out_cache = (uint8_t)(cur & 0x0F);
        h->out_valid = true;
    }

    uint8_t nxt = h->out_cache;
    if (level) nxt |=  (1u << bit);
    else       nxt &= ~(1u << bit);
    nxt &= 0x0F;

    // évite write si identique
    if (nxt == h->out_cache) {
        xSemaphoreGive(h->mutex);
        return ESP_OK;
    }

    uint8_t buf[2] = { PCA9536_REG_OUTPUT, nxt };
    esp_err_t ret = i2c_master_write_to_device(
        h->cfg.port, h->cfg.addr,
        buf, sizeof(buf),
        ms_to_ticks_(h->cfg.timeout_ms)
    );

    if (ret == ESP_OK) h->out_cache = nxt;

    xSemaphoreGive(h->mutex);

    if (ret != ESP_OK) ESP_LOGE(TAG, "Write OUTPUT failed: %s", esp_err_to_name(ret));
    return ret;
}

esp_err_t pca9536_set_output_mask(pca9536_handle_t *h, uint8_t mask, uint8_t value)
{
    if (!h) return ESP_ERR_INVALID_ARG;

    mask  &= 0x0F;
    value &= 0x0F;

    if (!h->mutex) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(h->mutex, ms_to_ticks_(h->cfg.timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // 1) Source "cur" : cache si valide, sinon lecture HW
    uint8_t cur = 0;

    if (h->out_valid) {
        cur = (uint8_t)(h->out_cache & 0x0F);
    } else {
        uint8_t reg = PCA9536_REG_OUTPUT;
        esp_err_t ret = i2c_master_write_read_device(
            h->cfg.port, h->cfg.addr,
            &reg, 1,
            &cur, 1,
            ms_to_ticks_(h->cfg.timeout_ms)
        );
        if (ret != ESP_OK) {
            xSemaphoreGive(h->mutex);
            ESP_LOGE(TAG, "Read OUTPUT failed: %s", esp_err_to_name(ret));
            return ret;
        }
        cur &= 0x0F;
        h->out_cache = cur;
        h->out_valid = true;
    }

    // 2) Compute next
    uint8_t nxt = (uint8_t)((cur & (uint8_t)~mask) | (value & mask));
    nxt &= 0x0F;

    // 3) Evite un write inutile
    if (nxt == cur) {
        xSemaphoreGive(h->mutex);
        return ESP_OK;
    }

    // 4) Write HW
    uint8_t buf[2] = { PCA9536_REG_OUTPUT, nxt };
    esp_err_t ret = i2c_master_write_to_device(
        h->cfg.port, h->cfg.addr,
        buf, sizeof(buf),
        ms_to_ticks_(h->cfg.timeout_ms)
    );

    if (ret == ESP_OK) {
        h->out_cache = nxt;
        h->out_valid = true;
    }

    xSemaphoreGive(h->mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write OUTPUT failed: %s", esp_err_to_name(ret));
    }
    return ret;
}


esp_err_t pca9536_set_outputs(pca9536_handle_t *h, uint8_t value)
{
    if (!h) return ESP_ERR_INVALID_ARG;
    value &= 0x0F;
    if (!h->mutex) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(h->mutex, ms_to_ticks_(h->cfg.timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // évite les writes inutiles
    if (h->out_valid && h->out_cache == value) {
        xSemaphoreGive(h->mutex);
        return ESP_OK;
    }

    uint8_t buf[2] = { PCA9536_REG_OUTPUT, value };
    esp_err_t ret = i2c_master_write_to_device(
        h->cfg.port, h->cfg.addr, buf, sizeof(buf),
        ms_to_ticks_(h->cfg.timeout_ms)
    );

    if (ret == ESP_OK) {
        h->out_cache = value;
        h->out_valid = true;
    }

    xSemaphoreGive(h->mutex);

    if (ret != ESP_OK) ESP_LOGE(TAG, "Write OUTPUT failed: %s", esp_err_to_name(ret));
    return ret;
}
