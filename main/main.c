#include "initialisation.h"
#include "test.h"
#include "esp_err.h"

// Example usage (app_main or init code)








void app_main(void)
{
    loratube_ctx_t ctx;
    loratube_init_config_t icfg = loratube_init_config_default();
    ESP_ERROR_CHECK(loratube_init(&ctx, &icfg));

    loratube_test_config_t tcfg = loratube_test_config_default();
    ESP_ERROR_CHECK(loratube_run_tests(&ctx, &tcfg));

}
