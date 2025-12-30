#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

// ===== I2C =====
#define PCF8523_ADDR              0x68

// ===== Register map (PCF8523) =====
#define PCF8523_REG_CONTROL_1     0x00
#define PCF8523_REG_CONTROL_2     0x01
#define PCF8523_REG_CONTROL_3     0x02

#define PCF8523_REG_SECONDS       0x03
#define PCF8523_REG_MINUTES       0x04
#define PCF8523_REG_HOURS         0x05
#define PCF8523_REG_DAYS          0x06
#define PCF8523_REG_WEEKDAYS      0x07
#define PCF8523_REG_MONTHS        0x08
#define PCF8523_REG_YEARS         0x09

#define PCF8523_REG_MIN_ALARM     0x0A
#define PCF8523_REG_HOUR_ALARM    0x0B
#define PCF8523_REG_DAY_ALARM     0x0C
#define PCF8523_REG_WDAY_ALARM    0x0D

#define PCF8523_REG_OFFSET        0x0E
#define PCF8523_REG_TMR_CLKOUT    0x0F

#define PCF8523_REG_TMR_A_FREQ    0x10
#define PCF8523_REG_TMR_A         0x11
#define PCF8523_REG_TMR_B_FREQ    0x12
#define PCF8523_REG_TMR_B         0x13

// ===== Control_1 (0x00) bits =====
#define PCF8523_CTRL1_CAP_SEL     (1u << 7)
#define PCF8523_CTRL1_T           (1u << 6)
#define PCF8523_CTRL1_STOP        (1u << 5)
#define PCF8523_CTRL1_SR          (1u << 4)
#define PCF8523_CTRL1_12_24       (1u << 3)
#define PCF8523_CTRL1_SIE         (1u << 2)
#define PCF8523_CTRL1_AIE         (1u << 1)
#define PCF8523_CTRL1_CIE         (1u << 0)

// ===== Control_2 (0x01) flags + enables =====
// flags (read in reg; clearing requires special handling)
#define PCF8523_CTRL2_WTAF        (1u << 7)
#define PCF8523_CTRL2_CTAF        (1u << 6)
#define PCF8523_CTRL2_CTBF        (1u << 5)
#define PCF8523_CTRL2_SF          (1u << 4)
#define PCF8523_CTRL2_AF          (1u << 3)
// interrupt enables
#define PCF8523_CTRL2_WTAIE       (1u << 2)
#define PCF8523_CTRL2_CTAIE       (1u << 1)
#define PCF8523_CTRL2_CTBIE       (1u << 0)

#define PCF8523_CTRL2_FLAGS_MASK  (PCF8523_CTRL2_WTAF | PCF8523_CTRL2_CTAF | PCF8523_CTRL2_CTBF | PCF8523_CTRL2_SF | PCF8523_CTRL2_AF)
#define PCF8523_CTRL2_IE_MASK     (PCF8523_CTRL2_WTAIE | PCF8523_CTRL2_CTAIE | PCF8523_CTRL2_CTBIE)

// ===== Tmr_CLKOUT_ctrl (0x0F) layout =====
// bits7..6 : TBC[1:0]  (Timer B enable/control)
// bits5..3 : COF[2:0]  (CLKOUT freq / disable)
// bits2..0 : TAC[2:0]  (Timer A enable/control)
#define PCF8523_TMRCLKOUT_TBC_MASK  (3u << 6)
#define PCF8523_TMRCLKOUT_COF_MASK  (7u << 3)
#define PCF8523_TMRCLKOUT_TAC_MASK  (7u << 0)

#define PCF8523_TMRCLKOUT_COF_SHIFT 3

// COF values
#define PCF8523_COF_32768HZ         0u  // COF=000
#define PCF8523_COF_16384HZ         1u
#define PCF8523_COF_8192HZ          2u
#define PCF8523_COF_4096HZ          3u
#define PCF8523_COF_1024HZ          4u
#define PCF8523_COF_32HZ            5u
#define PCF8523_COF_1HZ             6u
#define PCF8523_COF_DISABLE         7u  // COF=111 (CLKOUT disabled)

// Helper to compose COF field
#define PCF8523_TMRCLKOUT_COF(x)    (((uint8_t)(x) & 7u) << PCF8523_TMRCLKOUT_COF_SHIFT)

// "safe off": COF=disable + timers off -> 0x38 (commonly used)
#define PCF8523_TMRCLKOUT_ALL_OFF   ((uint8_t)0x38)

// ===== Datetime struct =====
typedef struct {
    uint16_t year;   // 2000..2099 typical
    uint8_t  month;  // 1..12
    uint8_t  day;    // 1..31
    uint8_t  wday;   // 0..6
    uint8_t  hour;   // 0..23
    uint8_t  min;    // 0..59
    uint8_t  sec;    // 0..59
} pcf8523_datetime_t;

typedef struct {
    i2c_port_t port;
    uint8_t    addr_7bit;
    uint32_t   timeout_ms;   // ms
    bool       initialized;
} pcf8523_t;

// ===== Basic I/O =====
esp_err_t pcf8523_init(pcf8523_t *dev, i2c_port_t port, uint8_t addr_7bit, uint32_t timeout_ms);
esp_err_t pcf8523_read_reg (pcf8523_t *dev, uint8_t reg, uint8_t *val);
esp_err_t pcf8523_write_reg(pcf8523_t *dev, uint8_t reg, uint8_t  val);
esp_err_t pcf8523_read     (pcf8523_t *dev, uint8_t start_reg, uint8_t *buf, size_t len);
esp_err_t pcf8523_write    (pcf8523_t *dev, uint8_t start_reg, const uint8_t *buf, size_t len);

// ===== RTC datetime =====
esp_err_t pcf8523_get_datetime(pcf8523_t *dev, pcf8523_datetime_t *dt);
esp_err_t pcf8523_set_datetime(pcf8523_t *dev, const pcf8523_datetime_t *dt);

// ===== CLKOUT / INT housekeeping =====
esp_err_t pcf8523_clkout_set(pcf8523_t *dev, uint8_t cof_value_0_7, bool log_readback);
esp_err_t pcf8523_clkout_disable(pcf8523_t *dev, bool log_readback);

// Clear flags in Control_2 (AF/SF/CTAF/CTBF/WTAF) without clobbering enables
esp_err_t pcf8523_clear_flags(pcf8523_t *dev, uint8_t flags_to_clear, bool log_readback);

// ===== Your legacy entry points (kept, but now correct) =====
// "debug init irq timer": makes a safe baseline (CLKOUT disabled + flags cleared + optional enable)
esp_err_t pcf8523_debug_init_irq_timer(pcf8523_t *dev, bool enable_timer_b_irq, bool log_readback);

// compatibility alias
static inline esp_err_t pcf8523_make_sqw_hi_z(pcf8523_t *dev, bool log_readback)
{
    return pcf8523_clkout_disable(dev, log_readback);
}

#ifdef __cplusplus
}
#endif
