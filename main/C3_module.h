#pragma once
#include "esp_check.h"
#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================
//  TEMP SENSOR (ESP32-C3 TSENS)
// =============================

typedef struct {
    // Utilise les enums ESP-IDF: TSENS_DAC_L0..L? (ex: TSENS_DAC_L3)
    int dac_offset;

    // nombre de tentatives de lecture (utile si tu veux du “best effort”)
    uint8_t read_retries;
} diag_tsens_cfg_t;

/**
 * @brief Démarre le capteur de température interne (TSENS).
 *
 * @param cfg peut être NULL -> configuration par défaut: dac_offset=TSENS_DAC_L3, read_retries=1
 */
esp_err_t diag_tsens_start(const diag_tsens_cfg_t *cfg);

/**
 * @brief Stoppe le capteur TSENS si démarré.
 */
esp_err_t diag_tsens_stop(void);

/**
 * @brief Lit température + raw.
 *
 * @param out_celsius peut être NULL
 * @param out_raw peut être NULL
 */
esp_err_t diag_tsens_read(float *out_celsius, uint32_t *out_raw);

/**
 * @brief Boucle de logs (bloquante): échantillons périodiques.
 *
 * @param tag Tag ESP_LOGI/ESP_LOGE
 * @param samples nombre de mesures
 * @param period_ms période entre mesures
 */
void diag_tsens_log_periodic(const char *tag, uint32_t samples, uint32_t period_ms);


// =============================
//  E22 GPIO STATE LOGGER
// =============================

typedef struct {
    gpio_num_t m0;
    gpio_num_t m1;
    gpio_num_t aux;
    gpio_num_t tx;
    gpio_num_t rx;
} e22_gpio_pins_t;

/**
 * @brief Log l'état des GPIO (M0/M1/AUX/TX/RX) pour un module E22.
 *
 * @param tag Tag de log
 * @param pins description des pins
 */
void e22_log_gpio_state(const char *tag, const e22_gpio_pins_t *pins);

// Optionnel: wrapper si ton projet définit déjà E22_M0_GPIO, etc.
#if defined(E22_M0_GPIO) && defined(E22_M1_GPIO) && defined(E22_AUX_GPIO) && defined(E22_TX_GPIO) && defined(E22_RX_GPIO)
#define E22_GPIO_PINS_DEFAULT() ((e22_gpio_pins_t){ \
    .m0  = (gpio_num_t)E22_M0_GPIO, \
    .m1  = (gpio_num_t)E22_M1_GPIO, \
    .aux = (gpio_num_t)E22_AUX_GPIO, \
    .tx  = (gpio_num_t)E22_TX_GPIO, \
    .rx  = (gpio_num_t)E22_RX_GPIO, \
})
#endif

#ifdef __cplusplus
}
#endif
