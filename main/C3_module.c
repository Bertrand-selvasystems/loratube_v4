/**
 * @file    diag_tsens.c
 * @brief   TSENS (ESP32-C3) + VSENSE (ADC) : start/stop/read + logs périodiques.
 *
 * @details
 * - TSENS : module *stateful* (un seul handle global). Lecture en °C ok.
 *           Le "raw" n’est pas exposé par l’API publique du driver récent → `ESP_ERR_NOT_SUPPORTED` si demandé.
 * - VSENSE : lecture ADC1 (GPIO1 -> ADC1_CHANNEL_1) avec moyenne/min/max pour stabilité (entrée haute impédance),
 *            conversion Vpack via polynôme `adc_to_voltage(Kmean)`, log bring-up : moyenne_raw / ecart_raw / Vpack.
 */

#include "C3_module.h"

#include <inttypes.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/gpio.h"
#include "driver/temperature_sensor.h"

// --- ADC legacy (simple, aligné avec ton code existant) ---
#include "driver/adc.h"
#include "esp_rom_sys.h"   // ets_delay_us

// =============================
//  TEMP SENSOR (ESP32-C3 TSENS)
// =============================

/** @brief Tag de log interne du module TSENS. */
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

// =============================
//  VSENSE / PACK VOLTAGE (ADC)
// =============================

/** @brief Tag de log interne du module VSENSE. */
static const char *TAG_VSENSE = "VSENSE";

/** @brief Vrai si l'ADC est configuré pour VSENSE. */
static bool s_vsense_started = false;



/** @brief Config active VSENSE. */
static diag_vsense_cfg_t s_vcfg = {
    .samples_per_read = 64,
    .intersample_delay_us = 500,
    .atten = ADC_ATTEN_DB_11,
    .width = ADC_WIDTH_BIT_12,
};

// ---- Conversion raw ADC -> Vpack (polyfit) ----
static inline double adc_to_voltage(double K)
{
    return -1.604208e-7 * K * K
           + 1.191807e-2 * K
           + 6.024925e-1;
}

static inline void vsense_clamp_cfg_(diag_vsense_cfg_t *c)
{
    if (!c) return;
    if (c->samples_per_read == 0) c->samples_per_read = 1;
    if (c->intersample_delay_us > 20000) c->intersample_delay_us = 20000; // garde-fou
}

// =============================
//  TSENS API
// =============================

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
 * @brief Log `samples` mesures TSENS avec un délai `period_ms` entre chaque.
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

// =============================
//  VSENSE API (ADC legacy)
// =============================

/**
 * @brief Configure ADC1 pour VSENSE (GPIO1 -> ADC1_CHANNEL_1).
 * @param[in] cfg Optionnel. Si non NULL, copie dans `s_vcfg` (samples clampé >=1).
 *
 * @note La calibration polynomiale adc_to_voltage() suppose une config stable (width/atten identiques).
 */
esp_err_t diag_vsense_start(const diag_vsense_cfg_t *cfg)
{
    if (cfg) {
        s_vcfg = *cfg;
        vsense_clamp_cfg_(&s_vcfg);
    }

    if (s_vsense_started) return ESP_OK;

    // Config globale ADC1
    adc1_config_width(s_vcfg.width);

    // Config canal VSENSE
    adc1_config_channel_atten(VSENSE_CHAN, s_vcfg.atten);

    s_vsense_started = true;
    return ESP_OK;
}

/**
 * @brief Stop VSENSE.
 *
 * @note Avec l'API legacy adc1_* il n'y a pas de "uninstall". On marque juste arrêté.
 */
esp_err_t diag_vsense_stop(void)
{
    s_vsense_started = false;
    return ESP_OK;
}

/**
 * @brief Lit VSENSE : N samples -> moyenne/min/max sur raw K + conversion Vpack via polyfit.
 * @param[out] out_vpack   Optionnel : Vpack (V) via adc_to_voltage(Kmean).
 * @param[out] out_k_mean  Optionnel : moyenne raw ADC.
 * @param[out] out_k_span  Optionnel : ecart_raw = k_max - k_min.
 * @return ESP_OK si ok, sinon erreur.
 */
esp_err_t diag_vsense_read(double *out_vpack, uint32_t *out_k_mean, uint32_t *out_k_span)
{
    if (!s_vsense_started) return ESP_ERR_INVALID_STATE;

    const uint32_t n = (s_vcfg.samples_per_read == 0) ? 1u : (uint32_t)s_vcfg.samples_per_read;

    uint32_t acc = 0;
    uint32_t kmin = 0xFFFFFFFFu;
    uint32_t kmax = 0;

    // Prime read (réduit l'effet "mémoire" du S/H, utile si haute impédance)
    (void)adc1_get_raw(VSENSE_CHAN);
    if (s_vcfg.intersample_delay_us) esp_rom_delay_us(s_vcfg.intersample_delay_us);

    for (uint32_t i = 0; i < n; i++) {
        int raw = adc1_get_raw(VSENSE_CHAN);
        uint32_t k = (raw < 0) ? 0u : (uint32_t)raw;

        acc += k;
        if (k < kmin) kmin = k;
        if (k > kmax) kmax = k;

        if (s_vcfg.intersample_delay_us) esp_rom_delay_us(s_vcfg.intersample_delay_us);
    }

    uint32_t kmean = acc / n;
    uint32_t kspan = (kmax >= kmin) ? (kmax - kmin) : 0;

    if (out_k_mean) *out_k_mean = kmean;
    if (out_k_span) *out_k_span = kspan;
    if (out_vpack)  *out_vpack  = adc_to_voltage((double)kmean);

    return ESP_OK;
}

/**
 * @brief Log `samples` mesures VSENSE avec un délai `period_ms` entre chaque.
 * @param[in] tag        Tag de log (default "ADC").
 * @param[in] samples    Nombre d’itérations.
 * @param[in] period_ms  Période en ms.
 *
 * Log format (bring-up) :
 *   "VSENSE gpio=1 chan=1 moyenne_raw=... ecart_raw=... Vpack=..."
 *
 * @warning Fonction bloquante (vTaskDelay) : appelle-la depuis une task de debug.
 */
void diag_vsense_log_periodic(const char *tag, uint32_t samples, uint32_t period_ms)
{
    if (!tag) tag = "ADC";

    for (uint32_t i = 0; i < samples; i++) {
        double v = 0.0;
        uint32_t kmean = 0, kspan = 0;

        esp_err_t err = diag_vsense_read(&v, &kmean, &kspan);

        if (err == ESP_OK) {
            ESP_LOGI(tag,
                     "VSENSE gpio=%d chan=%d moyenne_raw=%" PRIu32 " ecart_raw=%" PRIu32 " Vpack=%.3f",
                     (int)VSENSE_GPIO, (int)VSENSE_CHAN, kmean, kspan, v);
        } else {
            ESP_LOGE(tag, "diag_vsense_read() failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }
}

double diag_vsense_adc_to_voltage(double K)
{
    return adc_to_voltage(K);
}
