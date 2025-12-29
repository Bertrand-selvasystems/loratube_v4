#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PCA9536_ADDR       0x41

#define PCA9536_REG_INPUT  0x00
#define PCA9536_REG_OUTPUT 0x01
#define PCA9536_REG_POLINV 0x02
#define PCA9536_REG_CONFIG 0x03

#define PCA9536_GP0 0
#define PCA9536_GP1 1
#define PCA9536_GP2 2
#define PCA9536_GP3 3

#define PCA9536_BIT_E22_EN     PCA9536_GP0
#define PCA9536_BIT_LED_GREEN  PCA9536_GP1
#define PCA9536_BIT_LED_RED    PCA9536_GP2
#define PCA9536_BIT_BUCK_MODE  PCA9536_GP3

typedef struct {
    i2c_port_t port;
    uint8_t    addr;
    uint32_t   timeout_ms;
} pca9536_config_t;

typedef struct {
    pca9536_config_t cfg;
    bool             initialized;
    SemaphoreHandle_t mutex;
    uint8_t          out_cache;  // cache OUTPUT (bits 0..3)
    bool             out_valid;  // cache valide ?
} pca9536_handle_t;

pca9536_config_t pca9536_config_default(i2c_port_t port);

esp_err_t pca9536_init(pca9536_handle_t *h, const pca9536_config_t *cfg);
esp_err_t pca9536_deinit(pca9536_handle_t *h);

esp_err_t pca9536_read_reg (pca9536_handle_t *h, uint8_t reg, uint8_t *val);
esp_err_t pca9536_write_reg(pca9536_handle_t *h, uint8_t reg, uint8_t  val);

esp_err_t pca9536_set_output_bit (pca9536_handle_t *h, uint8_t bit, bool level);
esp_err_t pca9536_set_output_mask(pca9536_handle_t *h, uint8_t mask, uint8_t value);
esp_err_t pca9536_set_outputs(pca9536_handle_t *h, uint8_t value);


// Helpers "application"
static inline esp_err_t e22_set(pca9536_handle_t *h, bool on)
{
    return pca9536_set_output_bit(h, PCA9536_BIT_E22_EN, on);
}
static inline esp_err_t led_green_set(pca9536_handle_t *h, bool on)
{
    return pca9536_set_output_bit(h, PCA9536_BIT_LED_GREEN, on);
}
static inline esp_err_t led_red_set(pca9536_handle_t *h, bool on)
{
    return pca9536_set_output_bit(h, PCA9536_BIT_LED_RED, on);
}
static inline esp_err_t buck_mode_set(pca9536_handle_t *h, bool on)
{
    return pca9536_set_output_bit(h, PCA9536_BIT_BUCK_MODE, on);
}

#ifdef __cplusplus
}
#endif
