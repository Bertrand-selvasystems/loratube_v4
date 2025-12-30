#include "initialisation.h"
#include "test.h"
#include "pca_mgr_task.h"
#include "PCA9536_module.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "mcu_alive_twdt.h"
#include "mesure_task.h"
#include "rtc_task.h"

// Example usage (app_main or init code)

static const sw_wdt_cfg_t g_wdt_cfg = {
    .twdt_timeout_ms = 3000,
    .kick_period_ms  = 1000,
    .idle_core_mask  = 0x1,
    .trigger_panic   = true,
};

static const mesure_task_cfg_t g_mes_cfg = {
    .n_iter   = 5,
    .delay_ms = 50,
};




void app_main(void)
{


    loratube_ctx_t ctx;
    loratube_init_config_t icfg = loratube_init_config_default();
    ESP_ERROR_CHECK(loratube_init(&ctx, &icfg));

    loratube_test_config_t tcfg = loratube_test_config_default();
    ESP_ERROR_CHECK(loratube_run_tests(&ctx, &tcfg));

    // ===== RTC one-shot service =====
    // (loratube_init() doit avoir initialisé ctx.rtc et mis ctx.i2c_ready = true)
    rtc_task_cfg_t rtc_cfg = {
        .rtc = &ctx.rtc,
    };
    ESP_ERROR_CHECK(rtc_task_start(&rtc_cfg));

    ESP_ERROR_CHECK(pca_mgr_set_green(PCA_LED_BLINK_SLOW));
    ESP_ERROR_CHECK(pca_mgr_set_red(PCA_LED_BLINK_FAST));


    // lance le watchdog    
    ESP_ERROR_CHECK(sw_wdt_start(&g_wdt_cfg));

    // lancement de la task de mesure
    ESP_ERROR_CHECK(mesure_task_start(&g_mes_cfg));
    
}
