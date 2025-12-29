/**
 * @file pca_mgr_task.c
 * @brief Self-contained PCA9536 manager: queue + task + API.
 */

#include <stdint.h>
#include <stdbool.h>
#include "esp_log.h"
#include "esp_err.h"
#include "pca_mgr_task.h"

static const char *TAG = "PCA_MGR";

// ===== Message format (1 byte) =====
// bit0 : E22_RESET_PULSE (event)
// bit1 : LED1_ENABLE (apply LED1_MODE)
// bit2 : LED2_ENABLE (apply LED2_MODE)
// bit3 : BUCK_FORCE_ON (latched true)
// bit4..5 : LED1_MODE (valid if bit1=1)  -> GREEN
// bit6..7 : LED2_MODE (valid if bit2=1)  -> RED

#define MSG_E22_PULSE_BIT    (1u << 0)
#define MSG_LED1_EN_BIT      (1u << 1)
#define MSG_LED2_EN_BIT      (1u << 2)
#define MSG_BUCK_FORCE_BIT   (1u << 3)

#define MSG_LED1_MODE_SHIFT  4
#define MSG_LED2_MODE_SHIFT  6
#define MSG_LED_MODE_MASK    0x03u

// ===== Timing =====
#define TICK_MS              250u
#define E22_PULSE_MS         1000u

#define TICKS_FROM_MS(ms)    ((uint16_t)(((ms) + (TICK_MS - 1u)) / TICK_MS))
#define E22_PULSE_TICKS      (TICKS_FROM_MS(E22_PULSE_MS))

#define BLINK_SLOW_MS        2000u
#define BLINK_FAST_MS        500u
#define BLINK_SLOW_HALF_TICKS  (TICKS_FROM_MS(BLINK_SLOW_MS) / 2u)
#define BLINK_FAST_HALF_TICKS  (TICKS_FROM_MS(BLINK_FAST_MS) / 2u)

typedef enum {
    LED_MODE_OFF        = 0,
    LED_MODE_ON         = 1,
    LED_MODE_BLINK_SLOW = 2,
    LED_MODE_BLINK_FAST = 3,
} led_mode_t;

typedef struct {
    led_mode_t mode;
    uint16_t   cnt;
    bool       level;
} led_sm_t;

typedef struct {
    pca9536_handle_t *pca;
    QueueHandle_t     q;

    uint8_t  out_cache;
    bool     buck_force;
    uint16_t e22_pulse_cnt;

    led_sm_t led_green;
    led_sm_t led_red;
} pca_mgr_ctx_t;

// Single instance (module-owned)
static pca_mgr_ctx_t s;

/** Build patch messages */
static inline uint8_t msg_green_(uint8_t mode2)
{
    return (uint8_t)(MSG_LED1_EN_BIT | ((mode2 & 0x03u) << MSG_LED1_MODE_SHIFT));
}
static inline uint8_t msg_red_(uint8_t mode2)
{
    return (uint8_t)(MSG_LED2_EN_BIT | ((mode2 & 0x03u) << MSG_LED2_MODE_SHIFT));
}
static inline uint8_t msg_e22_pulse_(void) { return (uint8_t)MSG_E22_PULSE_BIT; }
static inline uint8_t msg_buck_force_(void){ return (uint8_t)MSG_BUCK_FORCE_BIT; }

static inline uint8_t clamp_nibble_(uint8_t v) { return (uint8_t)(v & 0x0F); }

static inline uint8_t set_bit4_(uint8_t out, uint8_t bit, bool on)
{
    if (on) out |=  (1u << bit);
    else    out &= ~(1u << bit);
    return clamp_nibble_(out);
}

static void led_apply_mode_(led_sm_t *s_led, led_mode_t m)
{
    s_led->mode = m;

    switch (m) {
    case LED_MODE_OFF:
        s_led->level = false;
        s_led->cnt   = 0;
        break;
    case LED_MODE_ON:
        s_led->level = true;
        s_led->cnt   = 0;
        break;
    case LED_MODE_BLINK_SLOW: {
        s_led->level = true;
        uint16_t half = BLINK_SLOW_HALF_TICKS;
        if (half == 0) half = 1;
        s_led->cnt = half;
    } break;
    case LED_MODE_BLINK_FAST: {
        s_led->level = true;
        uint16_t half = BLINK_FAST_HALF_TICKS;
        if (half == 0) half = 1;
        s_led->cnt = half;
    } break;
    default:
        s_led->mode  = LED_MODE_OFF;
        s_led->level = false;
        s_led->cnt   = 0;
        break;
    }
}

static void led_tick_(led_sm_t *s_led)
{
    if (s_led->mode == LED_MODE_BLINK_SLOW || s_led->mode == LED_MODE_BLINK_FAST) {
        if (s_led->cnt > 0) s_led->cnt--;
        if (s_led->cnt == 0) {
            s_led->level = !s_led->level;
            uint16_t half = (s_led->mode == LED_MODE_BLINK_SLOW) ? BLINK_SLOW_HALF_TICKS : BLINK_FAST_HALF_TICKS;
            if (half == 0) half = 1;
            s_led->cnt = half;
        }
    }
}

static esp_err_t write_out_if_changed_(uint8_t out)
{
    out = clamp_nibble_(out);
    if (s.out_cache == out) return ESP_OK;

    esp_err_t ret = pca9536_write_reg(s.pca, PCA9536_REG_OUTPUT, out);
    if (ret == ESP_OK) s.out_cache = out;
    return ret;
}

static void task_pca_mgr_(void *arg)
{
    (void)arg;

    // Deterministic initial output:
    // E22_EN=1, BUCK=0, LEDs off
    uint8_t out = 0;
    out = set_bit4_(out, PCA9536_BIT_E22_EN,    true);
    out = set_bit4_(out, PCA9536_BIT_BUCK_MODE, false);
    out = set_bit4_(out, PCA9536_BIT_LED_GREEN, false);
    out = set_bit4_(out, PCA9536_BIT_LED_RED,   false);

    (void)pca9536_write_reg(s.pca, PCA9536_REG_OUTPUT, out);
    s.out_cache = clamp_nibble_(out);

    led_apply_mode_(&s.led_green, LED_MODE_OFF);
    led_apply_mode_(&s.led_red,   LED_MODE_OFF);
    s.buck_force    = false;
    s.e22_pulse_cnt = 0;

    ESP_LOGI(TAG, "start tick=%ums e22_pulse=%ums", (unsigned)TICK_MS, (unsigned)E22_PULSE_MS);

    TickType_t last = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(TICK_MS));

        // 1) Drain queue (apply patches)
        uint8_t msg = 0;
        while (xQueueReceive(s.q, &msg, 0) == pdTRUE) {

            if (msg & MSG_E22_PULSE_BIT) {
                // Rearm pulse (keep longest)
                if (s.e22_pulse_cnt < E22_PULSE_TICKS) {
                    s.e22_pulse_cnt = E22_PULSE_TICKS;
                }
            }

            if (msg & MSG_BUCK_FORCE_BIT) {
                s.buck_force = true;
            }

            if (msg & MSG_LED1_EN_BIT) {
                led_mode_t m1 = (led_mode_t)((msg >> MSG_LED1_MODE_SHIFT) & MSG_LED_MODE_MASK);
                led_apply_mode_(&s.led_green, m1);
            }

            if (msg & MSG_LED2_EN_BIT) {
                led_mode_t m2 = (led_mode_t)((msg >> MSG_LED2_MODE_SHIFT) & MSG_LED_MODE_MASK);
                led_apply_mode_(&s.led_red, m2);
            }
        }

        // 2) Tick LED state machines
        led_tick_(&s.led_green);
        led_tick_(&s.led_red);

        // 3) Tick E22 pulse timer
        bool e22_en = true;
        if (s.e22_pulse_cnt > 0) {
            s.e22_pulse_cnt--;
            e22_en = false;
        }

        // 4) Compute OUTPUT nibble
        uint8_t out_next = s.out_cache; // preserve other bits (still 4-bit only here)
        out_next = set_bit4_(out_next, PCA9536_BIT_E22_EN,    e22_en);
        out_next = set_bit4_(out_next, PCA9536_BIT_BUCK_MODE, s.buck_force);
        out_next = set_bit4_(out_next, PCA9536_BIT_LED_GREEN, s.led_green.level);
        out_next = set_bit4_(out_next, PCA9536_BIT_LED_RED,   s.led_red.level);

        // 5) Write if changed
        esp_err_t ret = write_out_if_changed_(out_next);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "OUTPUT write failed: %s", esp_err_to_name(ret));
        }
    }
}

// ===== Public API =====

esp_err_t pca_mgr_init(pca9536_handle_t *pca)
{
    if (!pca || !pca->initialized) return ESP_ERR_INVALID_STATE;
    if (s.q != NULL) return ESP_ERR_INVALID_STATE; // already init

    s.pca = pca;

    s.q = xQueueCreate(8, sizeof(uint8_t));
    if (!s.q) return ESP_ERR_NO_MEM;

    BaseType_t ok = xTaskCreate(task_pca_mgr_, "task_pca_mgr", 2048, NULL, 1, NULL);
    if (ok != pdPASS) {
        vQueueDelete(s.q);
        s.q = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static inline esp_err_t send_(uint8_t msg)
{
    if (!s.q) return ESP_ERR_INVALID_STATE;
    return (xQueueSend(s.q, &msg, 0) == pdTRUE) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t pca_mgr_set_green(pca_led_mode_t mode)      { return send_(msg_green_((uint8_t)mode)); }
esp_err_t pca_mgr_set_red  (pca_led_mode_t mode)      { return send_(msg_red_((uint8_t)mode)); }
esp_err_t pca_mgr_pulse_e22_reset(void)               { return send_(msg_e22_pulse_()); }
esp_err_t pca_mgr_force_buck_on(void)                 { return send_(msg_buck_force_()); }

