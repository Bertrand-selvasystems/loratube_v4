#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/uart.h"

/* ---------- Types publics ---------- */

typedef enum
{
    E22_MODE_NORMAL = 0,  // M0=0 M1=0
    E22_MODE_WOR    = 1,  // M0=1 M1=0
    E22_MODE_SLEEP  = 2,  // M0=1 M1=1
    E22_MODE_CONFIG = 3,  // M0=0 M1=1
} e22_mode_t;

typedef struct
{
    uart_port_t uart_num;

    gpio_num_t tx_gpio;
    gpio_num_t rx_gpio;

    gpio_num_t m0_gpio;
    gpio_num_t m1_gpio;
    gpio_num_t aux_gpio;

    int uart_baud_config;
    int uart_rx_buf_size;

    uint32_t timeout_ms;
    uint32_t mode_recover_ms;
} e22_config_t;

typedef struct
{
    e22_config_t cfg;
    e22_mode_t   mode;
    bool         initialized;
} e22_handle_t;

/* ---------- API ---------- */

/**
 * @brief Retourne une config par défaut (pins, baud, timeouts…)
 */
e22_config_t e22_config_default(void);

/**
 * @brief Initialise GPIO + UART, force MODE_NORMAL au démarrage
 */
esp_err_t e22_init(e22_handle_t *h, const e22_config_t *cfg);

/**
 * @brief Désinstalle le driver UART
 */
esp_err_t e22_deinit(e22_handle_t *h);

/**
 * @brief Attend AUX=1 (stable), avec timeout en ms
 */
esp_err_t e22_wait_aux_high(e22_handle_t *h, uint32_t timeout_ms);

/**
 * @brief Change le mode (M0/M1), attend la stabilisation + AUX
 */
esp_err_t e22_set_mode(e22_handle_t *h, e22_mode_t mode);

/**
 * @brief Vide l'UART RX (drain)
 */
void e22_clear_rx(e22_handle_t *h);

/**
 * @brief Envoie un buffer brut sur UART (optionnellement attend AUX avant/après)
 * @return nb d'octets écrits, ou -1 si erreur
 */
int e22_write(e22_handle_t *h, const void *data, size_t len,
              uint32_t wait_aux_before_ms, uint32_t wait_aux_after_ms);

/**
 * @brief Lit jusqu'à maxlen octets depuis UART avec timeout
 * @return nb d'octets lus, ou -1 si erreur
 */
int e22_read(e22_handle_t *h, void *buf, size_t maxlen, uint32_t timeout_ms);

/**
 * @brief Lit les 7 octets de settings (commande C1 00 07) en MODE_CONFIG
 * @param out_params7 Peut être NULL
 */
esp_err_t e22_read_settings(e22_handle_t *h, uint8_t out_params7[7]);

/**
 * @brief Modifie les 2 bits de puissance TX (PACKET[1:0]) et écrit settings
 * @param power_bits 0..3 (au-delà -> clamp à 0)
 * @param permanent true => write 0xC0 (flash), false => 0xC2 (volatile)
 */
esp_err_t e22_set_tx_power(e22_handle_t *h, uint8_t power_bits, bool permanent);

/**
 * @brief Log l’état des GPIO (M0/M1/AUX/TX/RX)
 */
void e22_log_gpio_state(e22_handle_t *h, const char *tag);

/**
 * @brief Séquence de debug “sniff” UART en mode CONFIG pendant sniff_ms
 */
void e22_super_debug(e22_handle_t *h, uint32_t sniff_ms);

#ifdef __cplusplus
}
#endif
