#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PCF8523_ADDR 0x68

#define PCF8523_REG_CONTROL_2        0x01
#define PCF8523_REG_TMR_CLKOUT_CTRL  0x0F

// Control_2 bits
#define PCF8523_CTRL2_TI_TP   (1u << 7)
#define PCF8523_CTRL2_TF      (1u << 6)
#define PCF8523_CTRL2_AF      (1u << 5)
#define PCF8523_CTRL2_SI      (1u << 4)
#define PCF8523_CTRL2_MI      (1u << 3)
#define PCF8523_CTRL2_AIE     (1u << 2)
#define PCF8523_CTRL2_TIE     (1u << 1)
#define PCF8523_CTRL2_CTS     (1u << 0)

typedef struct {
    i2c_port_t port;
    uint8_t    addr_7bit;
    uint32_t   timeout_ms;   // <-- ms, pas TickType_t
    bool       initialized;
} pcf8523_t;

esp_err_t pcf8523_init(pcf8523_t *dev, i2c_port_t port, uint8_t addr_7bit, uint32_t timeout_ms);

esp_err_t pcf8523_read_reg (pcf8523_t *dev, uint8_t reg, uint8_t *val);
esp_err_t pcf8523_write_reg(pcf8523_t *dev, uint8_t reg, uint8_t  val);

esp_err_t pcf8523_debug_init_irq_timer(pcf8523_t *dev, bool log_readback);
esp_err_t pcf8523_make_sqw_hi_z(pcf8523_t *dev, bool log_readback);

#ifdef __cplusplus
}
#endif
