/**
 * @file    mesure_task.c
 * @brief   Mesure au réveil: TSENS + VSENSE, push dans 2 queues, puis set eventbit et auto-delete.
 *
 * Design:
 * - Zéro dépendance à des "cfg_t" exotiques.
 * - main() peut faire:
 *      mesure_task_cfg_t mcfg = { .n_iter=5, .delay_ms=50 };
 *      ESP_ERROR_CHECK(mesure_task_start(&mcfg));
 * - La config est copiée en statique (1 instance), donc pas de malloc.
 *
 * Compatibility:
 * - Uses C3_module public API:
 *      diag_tsens_start/read/stop
 *      diag_vsense_start/read/stop
 */

#include "mesure_task.h"
#include "system_queues.h"
#include "system_state.h"     // eg_state + EG_STATE_MEAS_DONE
#include "system_types.h"     // c3_temp_msg_t / c3_vbat_msg_t
#include "C3_module.h"        // diag_tsens_* / diag_vsense_*
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "MESURE";

/* Copie persistante de la config (1 seule instance de task attendue). */
static mesure_task_cfg_t g_cfg;

/* Dernières valeurs valides (utile si une lecture échoue). */
static float g_last_temp_c = 0.0f;
static float g_last_vbat_v = 0.0f;

static float read_temp_c_(void)
{
    float c = 0.0f;
    esp_err_t err = diag_tsens_read(&c, NULL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "TSENS read failed: %s (using last %.2fC)", esp_err_to_name(err), g_last_temp_c);
        return g_last_temp_c;
    }
    g_last_temp_c = c;
    return c;
}

static float read_vbat_v_(void)
{
    double vd = 0.0;
    uint32_t kmean = 0, kspan = 0;

    esp_err_t err = diag_vsense_read(&vd, &kmean, &kspan);
    (void)kmean;
    (void)kspan;

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "VSENSE read failed: %s (using last %.3fV)", esp_err_to_name(err), g_last_vbat_v);
        return g_last_vbat_v;
    }

    float v = (float)vd;
    g_last_vbat_v = v;
    return v;
}

static void mesure_task_(void *arg)
{
    (void)arg;

    if (!q_temp || !q_vbat) {
        ESP_LOGE(TAG, "queues not initialized");
        vTaskDelete(NULL);
        return;
    }
    if (!eg_state) {
        ESP_LOGE(TAG, "eg_state not initialized");
        vTaskDelete(NULL);
        return;
    }

    // Ensure sensor backends are started (idempotent in C3_module)
    esp_err_t err = diag_tsens_start(NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "diag_tsens_start failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    err = diag_vsense_start(NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "diag_vsense_start failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    c3_temp_msg_t tmsg = {0};
    c3_vbat_msg_t vmsg = {0};

    uint32_t n = (g_cfg.n_iter == 0) ? 1u : (uint32_t)g_cfg.n_iter;

    for (uint32_t i = 0; i < n; i++) {
        tmsg.c10 = (int16_t)lroundf(read_temp_c_() * 10.0f);
        vmsg.mv  = (uint16_t)lroundf(read_vbat_v_() * 1000.0f);

        /* Queues taille 1 => on écrase la dernière valeur. */
        (void)xQueueOverwrite(q_temp, &tmsg);
        (void)xQueueOverwrite(q_vbat, &vmsg);

        ESP_LOGI(TAG, "iter %u/%u: T=%.1f C, VBAT=%.3f V",
         (unsigned)(i + 1), (unsigned)n,
         (double)tmsg.c10 / 10.0,
         (double)vmsg.mv  / 1000.0);


        if (g_cfg.delay_ms) vTaskDelay(pdMS_TO_TICKS(g_cfg.delay_ms));
    }

    /* Signal "mesures terminées". */
    xEventGroupSetBits(eg_state, EG_STATE_MEAS_DONE);

    ESP_LOGI(TAG, "done -> EG_STATE_MEAS_DONE set, deleting task");
    vTaskDelete(NULL);
}

esp_err_t mesure_task_start(const mesure_task_cfg_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;

    /* Préconditions: queues + event group créés avant. */
    if (!q_temp || !q_vbat) return ESP_ERR_INVALID_STATE;
    if (!eg_state) return ESP_ERR_INVALID_STATE;

    /* Copie safe: main peut passer un cfg sur stack. */
    g_cfg = *cfg;

    BaseType_t ok = xTaskCreate(
        mesure_task_, "mesure_task",
        2048, NULL,
        tskIDLE_PRIORITY, NULL
    );

    return (ok == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}
