#include "E22_module.h"

#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG_E22 = "E22";

/* ---------- helpers internes ---------- */

static esp_err_t e22_gpio_init_(const e22_config_t *c)
{
    gpio_config_t io = {0};

    // M0/M1 outputs
    io.mode = GPIO_MODE_OUTPUT;
    io.intr_type = GPIO_INTR_DISABLE;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.pin_bit_mask = (1ULL << c->m0_gpio) | (1ULL << c->m1_gpio);
    ESP_ERROR_CHECK(gpio_config(&io));

    // AUX input
    io.mode = GPIO_MODE_INPUT;
    io.pin_bit_mask = (1ULL << c->aux_gpio);
    ESP_ERROR_CHECK(gpio_config(&io));

    return ESP_OK;
}

static esp_err_t e22_uart_init_(const e22_config_t *c)
{
    const uart_config_t uc = {
        .baud_rate = c->uart_baud_config,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_driver_install(c->uart_num, c->uart_rx_buf_size, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(c->uart_num, &uc));
    ESP_ERROR_CHECK(uart_set_pin(c->uart_num, c->tx_gpio, c->rx_gpio,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    return ESP_OK;
}

static esp_err_t e22_read_exact_(uart_port_t uart, uint8_t *buf, int len, uint32_t timeout_ms)
{
    int got = 0;
    const TickType_t tmo = pdMS_TO_TICKS(timeout_ms);

    while (got < len)
    {
        int n = uart_read_bytes(uart, buf + got, len - got, tmo);
        if (n <= 0)
            return ESP_ERR_TIMEOUT;
        got += n;
    }
    return ESP_OK;
}

static void e22_decode_and_log_(const uint8_t params[7])
{
    uint8_t ADDH    = params[0];
    uint8_t ADDL    = params[1];
    uint8_t NETID   = params[2];
    uint8_t SPEED   = params[3];
    uint8_t PACKET  = params[4];
    uint8_t CHANNEL = params[5];
    uint8_t OPTIONS = params[6];

    uint16_t address = ((uint16_t)ADDH << 8) | ADDL;

    uint8_t uart_baud = (SPEED >> 5) & 0x07;
    uint8_t parity    = (SPEED >> 3) & 0x03;
    uint8_t air_rate  = (SPEED >> 0) & 0x07;

    uint8_t packet_len = (PACKET >> 6) & 0x03;
    uint8_t rssi_amb   = (PACKET >> 5) & 0x01;
    uint8_t power      = (PACKET >> 0) & 0x03;

    uint8_t rssi_inpk  = (OPTIONS >> 7) & 0x01;
    uint8_t tx_mode    = (OPTIONS >> 6) & 0x01;
    uint8_t repeater   = (OPTIONS >> 5) & 0x01;
    uint8_t lbt        = (OPTIONS >> 4) & 0x01;
    uint8_t wor        = (OPTIONS >> 3) & 0x01;
    uint8_t wor_cycle  = (OPTIONS >> 0) & 0x07;

    ESP_LOGI(TAG_E22, "ADDR=0x%04X NETID=0x%02X CH=%u", address, NETID, CHANNEL);
    ESP_LOGI(TAG_E22, "SPEED: uart=%u parity=%u air=%u", uart_baud, parity, air_rate);
    ESP_LOGI(TAG_E22, "PACKET: len=%u rssi_amb=%u power=%u", packet_len, rssi_amb, power);
    ESP_LOGI(TAG_E22, "OPTIONS: rssi_inpkt=%u tx_mode=%u rpt=%u lbt=%u wor=%u cycle=%u",
             rssi_inpk, tx_mode, repeater, lbt, wor, wor_cycle);
}

/* ---------- API ---------- */

e22_config_t e22_config_default(void)
{
    e22_config_t c = {
        .uart_num          = UART_NUM_1,
        .tx_gpio           = GPIO_NUM_6,
        .rx_gpio           = GPIO_NUM_10,
        .m0_gpio           = GPIO_NUM_2,
        .m1_gpio           = GPIO_NUM_7,
        .aux_gpio          = GPIO_NUM_5,
        .uart_baud_config  = 9600,
        .uart_rx_buf_size  = 512,
        .timeout_ms        = 1000,
        .mode_recover_ms   = 40,
    };
    return c;
}

esp_err_t e22_init(e22_handle_t *h, const e22_config_t *cfg)
{
    if (!h || !cfg) return ESP_ERR_INVALID_ARG;
    memset(h, 0, sizeof(*h));
    h->cfg = *cfg;

    ESP_LOGI(TAG_E22, "init: uart=%d TX=GPIO%d RX=GPIO%d M0=GPIO%d M1=GPIO%d AUX=GPIO%d baud=%d",
             (int)h->cfg.uart_num, (int)h->cfg.tx_gpio, (int)h->cfg.rx_gpio,
             (int)h->cfg.m0_gpio, (int)h->cfg.m1_gpio, (int)h->cfg.aux_gpio,
             (int)h->cfg.uart_baud_config);

    ESP_RETURN_ON_ERROR(e22_gpio_init_(&h->cfg), TAG_E22, "gpio init failed");
    ESP_RETURN_ON_ERROR(e22_uart_init_(&h->cfg), TAG_E22, "uart init failed");

    h->mode = E22_MODE_NORMAL;
    h->initialized = true;

    // Force NORMAL au démarrage (comportement simple et stable)
    (void)e22_set_mode(h, E22_MODE_NORMAL);
    return ESP_OK;
}

esp_err_t e22_deinit(e22_handle_t *h)
{
    if (!h || !h->initialized) return ESP_ERR_INVALID_STATE;
    esp_err_t err = uart_driver_delete(h->cfg.uart_num);
    h->initialized = false;
    return err;
}

esp_err_t e22_wait_aux_high(e22_handle_t *h, uint32_t timeout_ms)
{
    if (!h || !h->initialized) return ESP_ERR_INVALID_STATE;

    int64_t t0 = esp_timer_get_time(); // us
    while ((esp_timer_get_time() - t0) < (int64_t)timeout_ms * 1000)
    {
        if (gpio_get_level(h->cfg.aux_gpio) == 1)
        {
            vTaskDelay(pdMS_TO_TICKS(2));
            if (gpio_get_level(h->cfg.aux_gpio) == 1)
            {
                vTaskDelay(pdMS_TO_TICKS(3));
                return ESP_OK;
            }
        }
        taskYIELD();
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t e22_set_mode(e22_handle_t *h, e22_mode_t mode)
{
    if (!h || !h->initialized) return ESP_ERR_INVALID_STATE;

    switch (mode)
    {
        case E22_MODE_NORMAL: gpio_set_level(h->cfg.m0_gpio, 0); gpio_set_level(h->cfg.m1_gpio, 0); break;
        case E22_MODE_WOR:    gpio_set_level(h->cfg.m0_gpio, 1); gpio_set_level(h->cfg.m1_gpio, 0); break;
        case E22_MODE_SLEEP:  gpio_set_level(h->cfg.m0_gpio, 1); gpio_set_level(h->cfg.m1_gpio, 1); break;
        case E22_MODE_CONFIG: gpio_set_level(h->cfg.m0_gpio, 0); gpio_set_level(h->cfg.m1_gpio, 1); break;
        default: return ESP_ERR_INVALID_ARG;
    }

    h->mode = mode;

    vTaskDelay(pdMS_TO_TICKS(h->cfg.mode_recover_ms));
    (void)e22_wait_aux_high(h, h->cfg.timeout_ms);
    return ESP_OK;
}

void e22_clear_rx(e22_handle_t *h)
{
    if (!h || !h->initialized) return;
    uint8_t dump[64];
    while (uart_read_bytes(h->cfg.uart_num, dump, sizeof(dump), pdMS_TO_TICKS(10)) > 0) { }
}

int e22_write(e22_handle_t *h, const void *data, size_t len,
              uint32_t wait_aux_before_ms, uint32_t wait_aux_after_ms)
{
    if (!h || !h->initialized || !data) return -1;

    if (wait_aux_before_ms > 0) (void)e22_wait_aux_high(h, wait_aux_before_ms);

    int w = uart_write_bytes(h->cfg.uart_num, (const char *)data, (int)len);
    uart_wait_tx_done(h->cfg.uart_num, pdMS_TO_TICKS(50));

    if (wait_aux_after_ms > 0) (void)e22_wait_aux_high(h, wait_aux_after_ms);

    return w;
}

int e22_read(e22_handle_t *h, void *buf, size_t maxlen, uint32_t timeout_ms)
{
    if (!h || !h->initialized || !buf) return -1;
    return uart_read_bytes(h->cfg.uart_num, buf, (uint32_t)maxlen, pdMS_TO_TICKS(timeout_ms));
}

/**
 * @brief Lit et affiche les paramètres internes du module E22
 *
 * Passe temporairement en mode CONFIG.
 *
 * @param h Handle E22
 * @param out_params7 Buffer optionnel de 7 octets
 * @return ESP_OK si succès
 */
esp_err_t e22_read_settings(e22_handle_t *h, uint8_t out_params7[7])
{
    if (!h || !h->initialized) return ESP_ERR_INVALID_STATE;

    uint8_t head[3] = {0};
    uint8_t params[7] = {0};

    // Commande read settings
    const uint8_t cmd[3] = {0xC1, 0x00, 0x07};

    // Mode CONFIG (recommandé)
    ESP_RETURN_ON_ERROR(e22_set_mode(h, E22_MODE_CONFIG), TAG_E22, "set mode CONFIG failed");
    (void)e22_wait_aux_high(h, h->cfg.timeout_ms);

    e22_clear_rx(h);

    uart_write_bytes(h->cfg.uart_num, (const char *)cmd, sizeof(cmd));
    uart_wait_tx_done(h->cfg.uart_num, pdMS_TO_TICKS(50));

    ESP_RETURN_ON_ERROR(e22_read_exact_(h->cfg.uart_num, head, 3, h->cfg.timeout_ms), TAG_E22, "read head timeout");
    ESP_RETURN_ON_ERROR(e22_read_exact_(h->cfg.uart_num, params, 7, h->cfg.timeout_ms), TAG_E22, "read params timeout");

    ESP_LOGI(TAG_E22, "HEAD: %02X %02X %02X", head[0], head[1], head[2]);
    if (head[0] != 0xC1 || head[1] != 0x00 || head[2] != 0x07)
    {
        ESP_LOGE(TAG_E22, "HEAD mismatch");
        (void)e22_set_mode(h, E22_MODE_NORMAL);
        return ESP_FAIL;
    }

    e22_decode_and_log_(params);
    if (out_params7) memcpy(out_params7, params, 7);

    (void)e22_set_mode(h, E22_MODE_NORMAL);
    return ESP_OK;
}

esp_err_t e22_set_tx_power(e22_handle_t *h, uint8_t power_bits, bool permanent)
{
    if (!h || !h->initialized) return ESP_ERR_INVALID_STATE;

    if (power_bits > 3) power_bits = 0;

    uint8_t head[3] = {0};
    uint8_t p[7] = {0};

    // 1) lire settings en CONFIG
    ESP_RETURN_ON_ERROR(e22_set_mode(h, E22_MODE_CONFIG), TAG_E22, "set CONFIG failed");
    (void)e22_wait_aux_high(h, h->cfg.timeout_ms);
    e22_clear_rx(h);

    const uint8_t rd[3] = {0xC1, 0x00, 0x07};
    uart_write_bytes(h->cfg.uart_num, (const char *)rd, sizeof(rd));
    uart_wait_tx_done(h->cfg.uart_num, pdMS_TO_TICKS(50));

    ESP_RETURN_ON_ERROR(e22_read_exact_(h->cfg.uart_num, head, 3, h->cfg.timeout_ms), TAG_E22, "read head failed");
    ESP_RETURN_ON_ERROR(e22_read_exact_(h->cfg.uart_num, p, 7, h->cfg.timeout_ms), TAG_E22, "read params failed");

    if (head[0] != 0xC1 || head[1] != 0x00 || head[2] != 0x07)
    {
        ESP_LOGE(TAG_E22, "Read settings failed before write (head=%02X %02X %02X)", head[0], head[1], head[2]);
        (void)e22_set_mode(h, E22_MODE_NORMAL);
        return ESP_FAIL;
    }

    uint8_t packet_old = p[4];
    uint8_t packet_new = (uint8_t)((packet_old & ~0x03u) | (power_bits & 0x03u));

    // 2) écrire settings (C0 permanent / C2 volatile)
    uint8_t mem = permanent ? 0xC0 : 0xC2;
    uint8_t wr[10] = {mem, 0x00, 0x07, p[0], p[1], p[2], p[3], packet_new, p[5], p[6]};

    e22_clear_rx(h);
    uart_write_bytes(h->cfg.uart_num, (const char *)wr, sizeof(wr));
    uart_wait_tx_done(h->cfg.uart_num, pdMS_TO_TICKS(50));
    (void)e22_wait_aux_high(h, 1500);

    // 3) lire ack (certains modules renvoient 0xC1, d'autres renvoient mem -> on tolère)
    memset(head, 0, sizeof(head));
    memset(p, 0, sizeof(p));

    ESP_RETURN_ON_ERROR(e22_read_exact_(h->cfg.uart_num, head, 3, h->cfg.timeout_ms), TAG_E22, "ack head timeout");
    ESP_RETURN_ON_ERROR(e22_read_exact_(h->cfg.uart_num, p, 7, h->cfg.timeout_ms), TAG_E22, "ack params timeout");

    bool head_ok = ((head[0] == 0xC1) || (head[0] == mem)) && (head[1] == 0x00) && (head[2] == 0x07);
    if (!head_ok)
    {
        ESP_LOGE(TAG_E22, "Write ack mismatch (head=%02X %02X %02X)", head[0], head[1], head[2]);
        (void)e22_set_mode(h, E22_MODE_NORMAL);
        return ESP_FAIL;
    }

    bool ok = (p[4] == packet_new);
    ESP_LOGI(TAG_E22, "TX power set: old=0x%02X new=0x%02X ack=0x%02X -> %s",
             packet_old, packet_new, p[4], ok ? "OK" : "FAIL");

    (void)e22_set_mode(h, E22_MODE_NORMAL);
    return ok ? ESP_OK : ESP_FAIL;
}

void e22_log_gpio_state(e22_handle_t *h, const char *tag)
{
    if (!h || !h->initialized) return;

    int lvl_m0  = gpio_get_level(h->cfg.m0_gpio);
    int lvl_m1  = gpio_get_level(h->cfg.m1_gpio);
    int lvl_aux = gpio_get_level(h->cfg.aux_gpio);
    int lvl_tx  = gpio_get_level(h->cfg.tx_gpio);
    int lvl_rx  = gpio_get_level(h->cfg.rx_gpio);

    ESP_LOGI(tag, "GPIO M0 = GPIO%d, level=%d", (int)h->cfg.m0_gpio, lvl_m0);
    ESP_LOGI(tag, "GPIO M1 = GPIO%d, level=%d", (int)h->cfg.m1_gpio, lvl_m1);
    ESP_LOGI(tag, "GPIO AUX= GPIO%d, level=%d", (int)h->cfg.aux_gpio, lvl_aux);
    ESP_LOGI(tag, "GPIO TX = GPIO%d, level=%d", (int)h->cfg.tx_gpio, lvl_tx);
    ESP_LOGI(tag, "GPIO RX = GPIO%d, level=%d", (int)h->cfg.rx_gpio, lvl_rx);
}

/**
 * @brief Active un mode de diagnostic avancé
 *
 * Dump :
 *  - états GPIO
 *  - trames UART reçues
 *  - comportement AUX
 *
 * @param h Handle E22
 * @param sniff_ms Durée d’écoute UART
 */
void e22_super_debug(e22_handle_t *h, uint32_t sniff_ms)
{
    if (!h || !h->initialized) return;

    ESP_LOGI(TAG_E22, "==== E22 SUPER DEBUG START ====");
    e22_log_gpio_state(h, TAG_E22);

    ESP_LOGI(TAG_E22, "Set MODE_CONFIG");
    (void)e22_set_mode(h, E22_MODE_CONFIG);
    (void)e22_wait_aux_high(h, h->cfg.timeout_ms);
    e22_log_gpio_state(h, TAG_E22);

    ESP_LOGI(TAG_E22, "Clear RX");
    e22_clear_rx(h);

    const uint8_t cmd[3] = {0xC1, 0x00, 0x07};
    ESP_LOGI(TAG_E22, "TX: C1 00 07");
    uart_write_bytes(h->cfg.uart_num, (const char *)cmd, sizeof(cmd));
    uart_wait_tx_done(h->cfg.uart_num, pdMS_TO_TICKS(50));

    uint8_t buf[32];
    int total = 0;
    int iter = 0;
    int64_t t_start = esp_timer_get_time();

    while ((esp_timer_get_time() - t_start) < (int64_t)sniff_ms * 1000)
    {
        int n = uart_read_bytes(h->cfg.uart_num, buf, sizeof(buf), pdMS_TO_TICKS(50));
        if (n > 0)
        {
            ESP_LOGI(TAG_E22, "[iter=%d] RX %d bytes:", iter, n);
            total += n;
            for (int i = 0; i < n; ++i)
            {
                ESP_LOGI(TAG_E22, "   byte[%02d] = 0x%02X ('%c')",
                         i, buf[i],
                         (buf[i] >= 32 && buf[i] < 127) ? buf[i] : '.');
            }
        }
        else
        {
            ESP_LOGI(TAG_E22, "[iter=%d] RX empty", iter);
        }
        iter++;
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_LOGI(TAG_E22, "Total RX bytes = %d", total);

    ESP_LOGI(TAG_E22, "Return MODE_NORMAL");
    (void)e22_set_mode(h, E22_MODE_NORMAL);
    e22_log_gpio_state(h, TAG_E22);

    ESP_LOGI(TAG_E22, "==== E22 SUPER DEBUG END ====");
}
