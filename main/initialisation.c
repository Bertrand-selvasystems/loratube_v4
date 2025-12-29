#include "initialisation.h"

#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "INIT";

// --- I2C init interne ---
static esp_err_t i2c_master_init_(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t freq_hz)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = freq_hz,
        .clk_flags = 0,
    };

    ESP_RETURN_ON_ERROR(i2c_param_config(port, &conf), TAG, "i2c_param_config failed");

    esp_err_t ret = i2c_driver_install(port, conf.mode, 0, 0, 0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C ready (port=%d SDA=GPIO%d SCL=GPIO%d freq=%" PRIu32 "Hz)",
             (int)port, (int)sda, (int)scl, freq_hz);
    return ESP_OK;
}

loratube_init_config_t loratube_init_config_default(void)
{
    loratube_init_config_t c;
    memset(&c, 0, sizeof(c));

    c.i2c_port = I2C_NUM_0;
    c.sda_gpio = GPIO_NUM_8;
    c.scl_gpio = GPIO_NUM_9;
    c.i2c_freq_hz = 100000;
    c.i2c_timeout_ms = 50;

    c.pca9536_addr = PCA9536_ADDR;
    c.pcf8523_addr = PCF8523_ADDR;
    c.fram_addr    = FRAM_I2C_ADDR;

    c.do_rtc_debug_irq_timer = true;
    c.force_rtc_sqw_hiz = true;

    c.blink_leds = true;
    c.blink_cycles = 4;
    c.blink_period_ms = 250;

    c.enable_e22_power = true;
    c.enable_buck_mode = true;

    c.e22_cfg = e22_config_default();

    c.pca_cfg = pca9536_config_default(I2C_NUM_0);
    return c;
}

esp_err_t loratube_led_green(loratube_ctx_t *ctx, bool on)
{
    if (!ctx) return ESP_ERR_INVALID_ARG;
    return led_green_set(&ctx->io, on);
}

esp_err_t loratube_led_red(loratube_ctx_t *ctx, bool on)
{
    if (!ctx) return ESP_ERR_INVALID_ARG;
    return led_red_set(&ctx->io, on);
}

esp_err_t loratube_e22_power(loratube_ctx_t *ctx, bool on)
{
    if (!ctx) return ESP_ERR_INVALID_ARG;
    return e22_set(&ctx->io, on);
}

esp_err_t loratube_buck_mode(loratube_ctx_t *ctx, bool on)
{
    if (!ctx) return ESP_ERR_INVALID_ARG;
    return buck_mode_set(&ctx->io, on);
}

static void blink_leds_(loratube_ctx_t *ctx, uint8_t cycles, uint32_t period_ms)
{
    for (uint8_t j = 0; j < cycles; ++j) {
        (void)loratube_led_green(ctx, true);
        (void)loratube_led_red(ctx, true);
        vTaskDelay(pdMS_TO_TICKS(period_ms));

        (void)loratube_led_green(ctx, false);
        (void)loratube_led_red(ctx, false);
        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }
}

esp_err_t loratube_e22_smoke_test(loratube_ctx_t *ctx)
{
    if (!ctx) return ESP_ERR_INVALID_ARG;

    uint8_t params[7] = {0};
    esp_err_t ret = e22_read_settings(&ctx->e22, params);
    if (ret != ESP_OK) {
        ESP_LOGW("E22", "smoke_test: read_settings failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI("E22", "smoke_test OK (params[0..6] read)");
    return ESP_OK;
}

esp_err_t loratube_fram_init_and_load(loratube_ctx_t *ctx, uint8_t fram_i2c_addr)
{
    if (!ctx) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = fram_init(ctx->cfg.i2c_port, fram_i2c_addr, ctx->cfg.i2c_timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE("FRAM", "fram_init failed (addr=0x%02X): %s", fram_i2c_addr, esp_err_to_name(ret));
        ctx->fram_ready = false;
        return ret;
    }

    fram_meta_source_t src = FRAM_META_SRC_A;
    ret = fram_meta_load(&ctx->fram_meta, &src);
    if (ret != ESP_OK) {
        ESP_LOGE("FRAM", "fram_meta_load failed: %s", esp_err_to_name(ret));
        ctx->fram_ready = false;
        return ret;
    }

    ctx->fram_ready = true;

    ESP_LOGI("FRAM", "FRAM OK, meta from %s, index=%u cap=%u logs flags=0x%02X",
             (src == FRAM_META_SRC_A) ? "A" : "B",
             (unsigned)ctx->fram_meta.index,
             (unsigned)fram_log_capacity(),
             (unsigned)ctx->fram_meta.flags);

    return ESP_OK;
}

esp_err_t loratube_init(loratube_ctx_t *ctx, const loratube_init_config_t *cfg)
{
    if (!ctx || !cfg) return ESP_ERR_INVALID_ARG;
    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;

    // 1) I2C bus
    ESP_RETURN_ON_ERROR(
        i2c_master_init_(cfg->i2c_port, cfg->sda_gpio, cfg->scl_gpio, cfg->i2c_freq_hz),
        TAG, "I2C init failed"
    );
    ctx->i2c_ready = true;

    // 2) PCA9536 init (nouvelle signature)
pca9536_config_t pca_cfg = {
    .port = cfg->i2c_port,
    .addr = cfg->pca9536_addr,
    .timeout_ms = cfg->i2c_timeout_ms,
};
ESP_RETURN_ON_ERROR(pca9536_init(&ctx->io, &pca_cfg), TAG, "PCA9536 init failed");

    if (cfg->blink_leds) {
        blink_leds_(ctx, cfg->blink_cycles, cfg->blink_period_ms);
    }

    // 3) PCF8523 init + config (tu gardes ton API existante)
    ESP_RETURN_ON_ERROR(
        pcf8523_init(&ctx->rtc, cfg->i2c_port, cfg->pcf8523_addr, cfg->i2c_timeout_ms),
        TAG, "PCF8523 init failed"
    );

    if (cfg->do_rtc_debug_irq_timer) {
        esp_err_t r = pcf8523_debug_init_irq_timer(&ctx->rtc, true);
        if (r == ESP_OK) ESP_LOGI("RTC", "PCF8523 IRQ timer open-drain (debug).");
        else ESP_LOGE("RTC", "PCF8523 debug irq_timer failed: %s", esp_err_to_name(r));
    }

    if (cfg->force_rtc_sqw_hiz) {
        esp_err_t r = pcf8523_make_sqw_hi_z(&ctx->rtc, true);
        if (r == ESP_OK) ESP_LOGI("RTC", "SQW forcée Hi-Z.");
        else ESP_LOGE("RTC", "PCF8523 Hi-Z failed: %s", esp_err_to_name(r));
    }

    // 4) Power rails via PCA9536
    if (cfg->enable_e22_power) {
        ESP_RETURN_ON_ERROR(loratube_e22_power(ctx, true), TAG, "E22 power ON failed");
    }
    if (cfg->enable_buck_mode) {
        ESP_RETURN_ON_ERROR(loratube_buck_mode(ctx, true), TAG, "Buck mode ON failed");
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    // 5) E22 init
    ESP_RETURN_ON_ERROR(e22_init(&ctx->e22, &cfg->e22_cfg), TAG, "E22 init failed");

    // initialisation des event bit
    ESP_RETURN_ON_ERROR(system_state_init(), TAG, "system_state_init failed");

    // 6) Smoke test
    (void)loratube_e22_smoke_test(ctx);

    // 7) FRAM init/load
    (void)loratube_fram_init_and_load(ctx, cfg->fram_addr);

    return ESP_OK;
}

esp_err_t loratube_deinit(loratube_ctx_t *ctx)
{
    if (!ctx) return ESP_ERR_INVALID_ARG;

    if (ctx->e22.initialized) {
        (void)e22_deinit(&ctx->e22);
    }

    if (ctx->i2c_ready) {
        esp_err_t r = i2c_driver_delete(ctx->cfg.i2c_port);
        if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "i2c_driver_delete: %s", esp_err_to_name(r));
            return r;
        }
        ctx->i2c_ready = false;
    }

    return ESP_OK;
}
