#include "initialisation.h"
#include "test.h"
//#include "pca_mgr_task.h"
#include "PCA9536_module.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
// Example usage (app_main or init code)


void app_main(void)
{
    
    loratube_ctx_t ctx;
    loratube_init_config_t icfg = loratube_init_config_default();
    ESP_ERROR_CHECK(loratube_init(&ctx, &icfg));

    loratube_test_config_t tcfg = loratube_test_config_default();
    ESP_ERROR_CHECK(loratube_run_tests(&ctx, &tcfg));

    // ESP_ERROR_CHECK(pca_mgr_set_green(PCA_LED_BLINK_SLOW));
    // ESP_ERROR_CHECK(pca_mgr_set_red(PCA_LED_BLINK_FAST));

}
