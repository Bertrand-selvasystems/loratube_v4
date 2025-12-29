/**
 * @file    diag_tsens.c
 * @brief   TSENS (ESP32-C3) : start/stop/read + log périodique via driver temperature_sensor (ESP-IDF).
 *
 * @details
 * Module *stateful* : un seul handle global. Lecture en °C ok.
 * Le "raw" n’est pas exposé par l’API publique du driver récent → `ESP_ERR_NOT_SUPPORTED` si demandé.
 */

#include "C3_module.h"

#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/temperature_sensor.h"

// =============================
//  TEMP SENSOR (ESP32-C3 TSENS)
// =============================

/** @brief Tag de log interne du module. */
static const char *TAG_TSENS = "TSENS";

/** @brief Vrai si le capteur est installé + activé. */
static bool s_tsens_started = false;

/** @brief Configuration active (copiée depuis `cfg` dans start). */
static diag_tsens_cfg_t s_cfg = {
    .dac_offset = 0,        // legacy : non utilisé par le nouveau driver
    .read_retries = 1,      // nb d’essais de lecture (>=1)
};

/** @brief Handle driver TSENS (NULL si non installé). */
static temperature_sensor_handle_t s_tsens = NULL;

/**
 * @brief Installe et active TSENS.
 * @param[in] cfg  Optionnel. Si non NULL, copie dans `s_cfg` (read_retries clampé à >=1).
 * @return ESP_OK si prêt (ou déjà démarré), sinon code d’erreur ESP-IDF.
 */
esp_err_t diag_tsens_start(const diag_tsens_cfg_t *cfg)
{
    if (cfg) {
        s_cfg = *cfg;
        if (s_cfg.read_retries == 0) s_cfg.read_retries = 1;
    }

    if (s_tsens_started) return ESP_OK;

    // Plage “raisonnable” pour le driver (ajuste si besoin)
    temperature_sensor_config_t ts_cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);

    esp_err_t err = temperature_sensor_install(&ts_cfg, &s_tsens);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_TSENS, "install failed: %s", esp_err_to_name(err));
        return err;
    }

    err = temperature_sensor_enable(s_tsens);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_TSENS, "enable failed: %s", esp_err_to_name(err));
        temperature_sensor_uninstall(s_tsens);
        s_tsens = NULL;
        return err;
    }

    s_tsens_started = true;
    return ESP_OK;
}

/**
 * @brief Désactive puis désinstalle TSENS.
 * @return ESP_OK si arrêté (ou déjà arrêté), sinon code d’erreur ESP-IDF.
 */
esp_err_t diag_tsens_stop(void)
{
    if (!s_tsens_started) return ESP_OK;

    esp_err_t err = temperature_sensor_disable(s_tsens);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_TSENS, "disable failed: %s", esp_err_to_name(err));
        return err;
    }

    err = temperature_sensor_uninstall(s_tsens);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_TSENS, "uninstall failed: %s", esp_err_to_name(err));
        return err;
    }

    s_tsens = NULL;
    s_tsens_started = false;
    return ESP_OK;
}

/**
 * @brief Lit TSENS.
 * @param[out] out_celsius  Optionnel : température (°C).
 * @param[out] out_raw      Optionnel : valeur brute. Non supportée ici → ESP_ERR_NOT_SUPPORTED si demandé.
 * @return
 * - ESP_OK si lecture °C ok et raw non demandé,
 * - ESP_ERR_INVALID_STATE si start non fait,
 * - ESP_ERR_NOT_SUPPORTED si raw demandé,
 * - autre erreur si lecture °C échoue après retries.
 *
 * @note Si out_raw != NULL et out_celsius != NULL, `out_celsius` peut être rempli mais le retour sera NOT_SUPPORTED.
 */
esp_err_t diag_tsens_read(float *out_celsius, uint32_t *out_raw)
{
    if (!s_tsens_started || !s_tsens) return ESP_ERR_INVALID_STATE;

    esp_err_t err = ESP_OK;

    if (out_celsius) {
        float t = 0.0f;
        esp_err_t last = ESP_FAIL;

        for (uint8_t k = 0; k < s_cfg.read_retries; k++) {
            last = temperature_sensor_get_celsius(s_tsens, &t);
            if (last == ESP_OK) break;
        }

        if (last != ESP_OK) err = last;
        else *out_celsius = t;
    }

    // Raw non exposé par l’API publique du driver récent
    if (out_raw) {
        *out_raw = 0;
        if (err == ESP_OK) err = ESP_ERR_NOT_SUPPORTED;
    }

    return err;
}

/**
 * @brief Log `samples` mesures avec un délai `period_ms` entre chaque.
 * @param[in] tag        Tag de log (default "TEMP").
 * @param[in] samples    Nombre d’itérations.
 * @param[in] period_ms  Période en ms.
 *
 * @warning Fonction bloquante (vTaskDelay) : appelle-la depuis une task de debug.
 */
void diag_tsens_log_periodic(const char *tag, uint32_t samples, uint32_t period_ms)
{
    if (!tag) tag = "TEMP";

    for (uint32_t i = 0; i < samples; i++) {
        float t_c = 0.0f;
        esp_err_t terr = diag_tsens_read(&t_c, NULL);

        if (terr == ESP_OK) {
            ESP_LOGI(tag, "ESP32-C3 internal temperature = %.2f °C", t_c);
        } else {
            ESP_LOGE(tag, "diag_tsens_read() failed: %s", esp_err_to_name(terr));
        }

        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }
}
