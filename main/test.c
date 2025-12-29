#include "test.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "FRAM_module.h"

static const char *TAG = "TEST";

static void blink_leds_(loratube_ctx_t *ctx, uint8_t cycles, uint32_t period_ms)
{
    for (uint8_t j = 0; j < cycles; ++j)
    {
        (void)loratube_led_green(ctx, true);
        (void)loratube_led_red(ctx, true);
        vTaskDelay(pdMS_TO_TICKS(period_ms));

        (void)loratube_led_green(ctx, false);
        (void)loratube_led_red(ctx, false);
        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }
}

static void vsense_loop_(const loratube_test_config_t *t)
{
    // Config ADC (API "legacy" comme ton main.c)
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(t->vsense_chan, t->vsense_atten);

    for (uint32_t i = 0; i < t->vsense_samples; ++i)
    {
        int raw = adc1_get_raw(t->vsense_chan);
        ESP_LOGI("ADC", "VSENSE raw=%d / 4095", raw);

        float v_adc = (t->adc_full_scale_v * (float)raw) / 4095.0f;
        ESP_LOGI("ADC", "VSENSE ~ %.3f V sur IO", v_adc);

        vTaskDelay(pdMS_TO_TICKS(t->vsense_period_ms));
    }
}

loratube_test_config_t loratube_test_config_default(void)
{
    loratube_test_config_t t = {0};

    // RTC
    t.rtc_debug_init_irq_timer = true;
    t.rtc_force_sqw_hiz = true;
    t.rtc_log_readback = true;

    // LEDs
    t.blink_leds = true;
    t.blink_cycles = 4;
    t.blink_period_ms = 250;

    // Rails
    t.set_e22_power_on = true;
    t.set_buck_mode_on = true;

    // E22
    t.e22_smoke_test = true;
    t.e22_set_tx_power = true;
    t.e22_tx_power_bits = 0;          // MAX
    t.e22_tx_power_permanent = true;  // permanent
    t.e22_set_mode = true;
    t.e22_mode = E22_MODE_WOR;
    t.e22_mode_hold_ms = 2000;

    // VSENSE (désactivé par défaut, comme ton code commenté)
    t.vsense_read_enable = false;
    t.vsense_chan = ADC1_CHANNEL_1;
    t.vsense_atten = ADC_ATTEN_DB_11;
    t.vsense_samples = 16000;
    t.vsense_period_ms = 1000;
    t.adc_full_scale_v = 3.3f;

    // Deep sleep (désactivé par défaut)
    t.go_deep_sleep = false;
    t.deep_sleep_delay_ms = 100;

    return t;
}

esp_err_t loratube_run_tests(loratube_ctx_t *ctx, const loratube_test_config_t *t)
{
    if (!ctx || !t) return ESP_ERR_INVALID_ARG;

    // --- RTC tests ---
    if (t->rtc_debug_init_irq_timer)
    {
        esp_err_t err = pcf8523_debug_init_irq_timer(&ctx->rtc, t->rtc_log_readback);
        if (err == ESP_OK)
            ESP_LOGI("RTC", "PCF8523 configuré en IRQ timer open-drain (diagnostic passé).");
        else
            ESP_LOGE("RTC", "PCF8523 init/diag échec: %s (vérifie VDD_RTC, 0x68, câblage INT).", esp_err_to_name(err));
    }

    if (t->rtc_force_sqw_hiz)
    {
        esp_err_t err = pcf8523_make_sqw_hi_z(&ctx->rtc, t->rtc_log_readback);
        if (err == ESP_OK)
            ESP_LOGI("RTC", "SQW forcée Hi-Z.");
        else
            ESP_LOGE("RTC", "Echec sequence Hi-Z: %s", esp_err_to_name(err));
    }

    // --- PCA9536 tests ---
    if (t->blink_leds)
    {
        blink_leds_(ctx, t->blink_cycles, t->blink_period_ms);
    }

    // --- Rails via PCA9536 ---
    if (t->set_e22_power_on)
    {
        ESP_RETURN_ON_ERROR(loratube_e22_power(ctx, true), TAG, "E22 power ON failed");
    }
    if (t->set_buck_mode_on)
    {
        ESP_RETURN_ON_ERROR(loratube_buck_mode(ctx, true), TAG, "Buck mode ON failed");
    }

    // --- E22 tests ---
    if (t->e22_smoke_test)
    {
        esp_err_t err = loratube_e22_smoke_test(ctx);
        if (err != ESP_OK)
            ESP_LOGW("E22", "Le test E22 a échoué: %s", esp_err_to_name(err));
        else
            ESP_LOGI("E22", "Le test E22 a réussi.");
    }

    if (t->e22_set_tx_power)
    {
        ESP_ERROR_CHECK(e22_set_tx_power(&ctx->e22, t->e22_tx_power_bits, t->e22_tx_power_permanent));
    }

    if (t->e22_set_mode)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP_ERROR_CHECK(e22_set_mode(&ctx->e22, t->e22_mode));
        vTaskDelay(pdMS_TO_TICKS(t->e22_mode_hold_ms));
    }

// Scratch FRAM test (non destructif)
ESP_ERROR_CHECK(loratube_test_fram_scratch(ctx, 0x0020));

    // --- VSENSE ADC test ---
    if (t->vsense_read_enable)
    {
        vsense_loop_(t);
    }

    // --- Deep sleep ---
    if (t->go_deep_sleep)
    {
        ESP_LOGI("MAIN", "Deep sleep now.");
        vTaskDelay(pdMS_TO_TICKS(t->deep_sleep_delay_ms));
        esp_deep_sleep_start();
    }

    return ESP_OK;
}



esp_err_t loratube_test_fram_scratch(loratube_ctx_t *ctx, uint16_t scratch_addr)
{
    if (!ctx) return ESP_ERR_INVALID_ARG;

    if (!ctx->fram_ready) {
        ESP_LOGW("FRAM", "FRAM not ready (skip test)");
        return ESP_ERR_INVALID_STATE;
    }

    // Évite la zone meta A et meta B
    if (scratch_addr < 0x0010) {
        ESP_LOGE("FRAM", "scratch_addr too low (0x%04X). Avoid meta area.", scratch_addr);
        return ESP_ERR_INVALID_ARG;
    }
    if (scratch_addr > (FRAM_SIZE_BYTES - 16)) {
        ESP_LOGE("FRAM", "scratch_addr too high (0x%04X).", scratch_addr);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t backup[16];
    uint8_t pattern[16];
    uint8_t readback[16];

    // 1) Backup
    esp_err_t ret = fram_read_bytes(scratch_addr, backup, sizeof(backup));
    if (ret != ESP_OK) {
        ESP_LOGE("FRAM", "backup read failed @0x%04X: %s", scratch_addr, esp_err_to_name(ret));
        return ret;
    }

    // 2) Write pattern
    for (int i = 0; i < 16; i++) pattern[i] = (uint8_t)(0xA5u ^ (uint8_t)i);

    ret = fram_write_bytes(scratch_addr, pattern, sizeof(pattern));
    if (ret != ESP_OK) {
        ESP_LOGE("FRAM", "pattern write failed @0x%04X: %s", scratch_addr, esp_err_to_name(ret));
        // tente restore
        (void)fram_write_bytes(scratch_addr, backup, sizeof(backup));
        return ret;
    }

    // 3) Readback compare
    ret = fram_read_bytes(scratch_addr, readback, sizeof(readback));
    if (ret != ESP_OK) {
        ESP_LOGE("FRAM", "readback failed @0x%04X: %s", scratch_addr, esp_err_to_name(ret));
        (void)fram_write_bytes(scratch_addr, backup, sizeof(backup));
        return ret;
    }

    if (memcmp(readback, pattern, sizeof(pattern)) != 0) {
        ESP_LOGE("FRAM", "compare FAILED @0x%04X", scratch_addr);
        (void)fram_write_bytes(scratch_addr, backup, sizeof(backup));
        return ESP_ERR_INVALID_RESPONSE;
    }

    // 4) Restore
    ret = fram_write_bytes(scratch_addr, backup, sizeof(backup));
    if (ret != ESP_OK) {
        ESP_LOGE("FRAM", "restore FAILED @0x%04X: %s", scratch_addr, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI("FRAM", "Scratch test OK @0x%04X (write/read/restore)", scratch_addr);
    return ESP_OK;
}

