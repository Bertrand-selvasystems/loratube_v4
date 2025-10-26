// Programme de test du PCB V3 LORATUBE

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include <string.h>

#define I2C_PORT I2C_NUM_0
#define E22_TX_ON_MS 5000
#define E22_TX_OFF_MS 10000
#define SDA_GPIO 8
#define SCL_GPIO 9
#define PCA9536_ADDR 0x41
#define PWRCONTROL_GPIO GPIO_NUM_2 // IO02
#define E22_UART_NUM UART_NUM_1
#define E22_TX_GPIO (GPIO_NUM_6)
#define E22_RX_GPIO (GPIO_NUM_4)
#define E22_M0_GPIO (GPIO_NUM_0)
#define E22_M1_GPIO (GPIO_NUM_7)
#define E22_AUX_GPIO (GPIO_NUM_10)
static const char *TAG_E22 = "E22";

// Delais (ms)
#define MODE_RECOVER_MS 25
#define RECOVERY_AFTER_WOR_MS 90
#define RECOVERY_AFTER_SLEEP_MS 850
#define E22_TIMEOUT_MS 1000

typedef enum
{
    E22_MODE_NORMAL = 0, // M0=0, M1=0
    E22_MODE_WOR = 1,    // M0=1, M1=0
    E22_MODE_SLEEP = 2,  // M0=1, M1=1
    E22_MODE_CONFIG = 3  // M0=0, M1=1
} e22_mode_t;

static esp_err_t e22_wait_aux_high(uint32_t timeout_ms)
{
    int64_t t0 = esp_timer_get_time(); // us
    while ((esp_timer_get_time() - t0) < (int64_t)timeout_ms * 1000)
    {
        if (gpio_get_level(E22_AUX_GPIO) == 1)
        {
            // petite double-validation façon Arduino
            vTaskDelay(pdMS_TO_TICKS(2));
            if (gpio_get_level(E22_AUX_GPIO) == 1)
            {
                vTaskDelay(pdMS_TO_TICKS(3));
                return ESP_OK;
            }
        }
        taskYIELD();
    }
    return ESP_ERR_TIMEOUT;
}

static void e22_set_mode(e22_mode_t mode)
{
    switch (mode)
    {
    case E22_MODE_NORMAL:
        gpio_set_level(E22_M0_GPIO, 0);
        gpio_set_level(E22_M1_GPIO, 0);
        break;
    case E22_MODE_WOR:
        gpio_set_level(E22_M0_GPIO, 1);
        gpio_set_level(E22_M1_GPIO, 0);
        break;
    case E22_MODE_SLEEP:
        gpio_set_level(E22_M0_GPIO, 1);
        gpio_set_level(E22_M1_GPIO, 1);
        break;
    case E22_MODE_CONFIG:
        gpio_set_level(E22_M0_GPIO, 0);
        gpio_set_level(E22_M1_GPIO, 1);
        break;
    }
    vTaskDelay(pdMS_TO_TICKS(MODE_RECOVER_MS));
    (void)e22_wait_aux_high(E22_TIMEOUT_MS);
}

static void e22_gpio_init(void)
{
    gpio_config_t io = {0};

    // M0/M1 en sorties
    io.mode = GPIO_MODE_OUTPUT;
    io.intr_type = GPIO_INTR_DISABLE;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;

    io.pin_bit_mask = (1ULL << E22_M0_GPIO) | (1ULL << E22_M1_GPIO);
    gpio_config(&io);

    // AUX en entrée
    io.mode = GPIO_MODE_INPUT;
    io.pin_bit_mask = (1ULL << E22_AUX_GPIO);
    gpio_config(&io);
}

static esp_err_t e22_uart_init(void)
{
    const uart_config_t uc = {
        .baud_rate = 9600, // vitesse de config E22
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, // 8N1
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        .source_clk = UART_SCLK_DEFAULT,
#endif
    };
    ESP_ERROR_CHECK(uart_driver_install(E22_UART_NUM, 512, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(E22_UART_NUM, &uc));
    ESP_ERROR_CHECK(uart_set_pin(E22_UART_NUM, E22_TX_GPIO, E22_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    return ESP_OK;
}

static void e22_clear_rx(void)
{
    uint8_t dump[64];
    while (uart_read_bytes(E22_UART_NUM, dump, sizeof(dump), 10 / portTICK_PERIOD_MS) > 0)
    {
    }
}

/**
 * Lecture registres locaux : envoie C1 00 07 et lit 3+7 octets.
 * Logue les champs et retourne ESP_OK si l’entête est conforme.
 */
static esp_err_t e22_read_settings_once(void)
{
    uint8_t head[3] = {0};
    uint8_t params[7] = {0};

    e22_clear_rx();

    const uint8_t cmd[3] = {0xC1, 0x00, 0x07};
    uart_write_bytes(E22_UART_NUM, (const char *)cmd, sizeof(cmd));
    uart_wait_tx_done(E22_UART_NUM, pdMS_TO_TICKS(20));

    int n = uart_read_bytes(E22_UART_NUM, head, 3, pdMS_TO_TICKS(600)); // entête
    if (n != 3)
    {
        ESP_LOGE(TAG_E22, "HEAD read failed (%d)", n);
        return ESP_FAIL;
    }

    n = uart_read_bytes(E22_UART_NUM, params, 7, pdMS_TO_TICKS(600)); // 7 regs
    if (n != 7)
    {
        ESP_LOGE(TAG_E22, "PARAMS read failed (%d)", n);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG_E22, "HEAD: %02X %02X %02X", head[0], head[1], head[2]);
    if (head[0] != 0xC1 || head[1] != 0x00 || head[2] != 0x07)
    {
        ESP_LOGE(TAG_E22, "HEAD mismatch");
        return ESP_FAIL;
    }

    // Décode rapide. Tout ca provisoire, pour aller vite au test...
    uint8_t ADDH = params[0], ADDL = params[1], NETID = params[2];
    uint8_t SPEED = params[3], PACKET = params[4], CHANNEL = params[5], OPTIONS = params[6];

    uint16_t address = ((uint16_t)ADDH << 8) | ADDL;
    uint8_t uart_baud = (SPEED >> 5) & 0x07;
    uint8_t parity = (SPEED >> 3) & 0x03;
    uint8_t air_rate = (SPEED) & 0x07;

    uint8_t packet_len = (PACKET >> 6) & 0x03;
    uint8_t rssi_amb = (PACKET >> 5) & 0x01;
    uint8_t power = (PACKET) & 0x03;

    uint8_t rssi_inpk = (OPTIONS >> 7) & 0x01;
    uint8_t tx_mode = (OPTIONS >> 6) & 0x01;
    uint8_t repeater = (OPTIONS >> 5) & 0x01;
    uint8_t lbt = (OPTIONS >> 4) & 0x01;
    uint8_t wor = (OPTIONS >> 3) & 0x01;
    uint8_t wor_cycle = (OPTIONS) & 0x07;

    ESP_LOGI(TAG_E22, "ADDR=0x%04X NETID=0x%02X CH=%u", address, NETID, CHANNEL);
    ESP_LOGI(TAG_E22, "SPEED: uart=%u parity=%u air=%u", uart_baud, parity, air_rate);
    ESP_LOGI(TAG_E22, "PACKET: len=%u rssi_amb=%u power=%u", packet_len, rssi_amb, power);
    ESP_LOGI(TAG_E22, "OPTIONS: rssi_inpkt=%u tx_mode=%u rpt=%u lbt=%u wor=%u cycle=%u",
             rssi_inpk, tx_mode, repeater, lbt, wor, wor_cycle);

    return ESP_OK;
}

static void e22_burst_tx_task(void *arg)
{
    static const char payload[] = "LT3-PING\n";
    for (;;)
    {
        // Phase ON — 5 s d'émission espacée
        e22_set_mode(E22_MODE_NORMAL);
        int64_t t0 = esp_timer_get_time(); // μs
        while ((esp_timer_get_time() - t0) < (int64_t)E22_TX_ON_MS * 1000)
        {
            // Attendre que le module soit prêt (AUX = HIGH), sans s’acharner
            (void)e22_wait_aux_high(500);
            // Envoyer un petit paquet
            uart_write_bytes(E22_UART_NUM, payload, sizeof(payload) - 1);
            // attendre la fin d’envoi UART
            uart_wait_tx_done(E22_UART_NUM, pdMS_TO_TICKS(20));

            // Petite respiration pour ne pas saturer (explique le décalage entre le courant moyen estimé via la chute de tension à la supercapa et le courant connu pour le module)
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        // Phase OFF 10 s de silence radio
        vTaskDelay(pdMS_TO_TICKS(E22_TX_OFF_MS));
    }
}

// ---- Régler la puissance TX : 0=MAX, 1=HIGH, 2=MID, 3=LOW (et pas l'inverse) ----
// permanent=false  -> écriture volatile (C2)
// permanent=true   -> écriture permanente (C0)
static esp_err_t e22_set_tx_power(uint8_t power_bits /*0..3*/, bool permanent)
{
    if (power_bits > 3)
        power_bits = 0; // clamp à MAX par défaut
    uint8_t head[3] = {0};
    uint8_t p[7] = {0};

    // 1) Lire l'état actuel
    e22_set_mode(E22_MODE_CONFIG);
    (void)e22_wait_aux_high(E22_TIMEOUT_MS);
    e22_clear_rx();

    const uint8_t rd[3] = {0xC1, 0x00, 0x07};
    uart_write_bytes(E22_UART_NUM, (const char *)rd, sizeof(rd));
    int n = uart_read_bytes(E22_UART_NUM, head, 3, 600 / portTICK_PERIOD_MS);
    n += uart_read_bytes(E22_UART_NUM, p, 7, 600 / portTICK_PERIOD_MS);
    if (n != 10 || head[0] != 0xC1 || head[1] != 0x00 || head[2] != 0x07)
    {
        ESP_LOGE(TAG_E22, "Read settings failed before write (n=%d, head=%02X %02X %02X)", n, head[0], head[1], head[2]);
        e22_set_mode(E22_MODE_NORMAL);
        return ESP_FAIL;
    }

    uint8_t packet_old = p[4];
    uint8_t packet_new = (uint8_t)((packet_old & ~0x03u) | (power_bits & 0x03u)); // bits[1:0] = power

    // 2) Construire trame d'écriture
    uint8_t mem = permanent ? 0xC0 : 0xC2; // C0=permanent, C2=temporaire
    uint8_t wr[10] = {mem, 0x00, 0x07, p[0], p[1], p[2], p[3], packet_new, p[5], p[6]};

    e22_clear_rx();
    uart_write_bytes(E22_UART_NUM, (const char *)wr, sizeof(wr));
    uart_wait_tx_done(E22_UART_NUM, pdMS_TO_TICKS(20));
    (void)e22_wait_aux_high(1500);

    // 3) Lire l'ACK et vérifier
    memset(head, 0, sizeof(head));
    memset(p, 0, sizeof(p));
    n = uart_read_bytes(E22_UART_NUM, head, 3, 800 / portTICK_PERIOD_MS);
    n += uart_read_bytes(E22_UART_NUM, p, 7, 800 / portTICK_PERIOD_MS);
    if (n != 10 || head[0] != 0xC1 || head[1] != 0x00 || head[2] != 0x07)
    {
        ESP_LOGE(TAG_E22, "Write ack mismatch (n=%d, head=%02X %02X %02X)", n, head[0], head[1], head[2]);
        e22_set_mode(E22_MODE_NORMAL);
        return ESP_FAIL;
    }

    bool ok = (p[4] == packet_new);
    ESP_LOGI(TAG_E22, "TX power set: old=0x%02X new=0x%02X ack=0x%02X -> %s",
             packet_old, packet_new, p[4], ok ? "OK" : "FAIL");

    e22_set_mode(E22_MODE_NORMAL);
    return ok ? ESP_OK : ESP_FAIL;
}

/** Séquence “smoke test” : MODE_CONFIG -> read settings -> MODE_NORMAL */
static esp_err_t e22_smoke_test(void)
{
    e22_gpio_init();
    e22_uart_init();

    // Optionnel : force NORMAL au départ, puis CONFIG
    e22_set_mode(E22_MODE_NORMAL);
    vTaskDelay(pdMS_TO_TICKS(30));

    e22_set_mode(E22_MODE_CONFIG);
    if (e22_wait_aux_high(E22_TIMEOUT_MS) != ESP_OK)
    {
        ESP_LOGW(TAG_E22, "AUX timeout in CONFIG; continue anyway");
    }

    esp_err_t ok = e22_read_settings_once();

    // Retour NORMAL
    e22_set_mode(E22_MODE_NORMAL);
    if (ok == ESP_OK)
        ESP_LOGI(TAG_E22, "E22 link OK (read settings passed)");
    else
        ESP_LOGE(TAG_E22, "E22 link FAIL (read settings failed)");
    return ok;
}

// --- PCF8523 : désactiver CLKOUT et activer l'IRQ en open-drain (mode Timer, pulse bref) ---
static esp_err_t pcf8523_debug_init_irq_timer(void)
{
    const uint8_t PCF = 0x68;
    esp_err_t ret;
    uint8_t reg, val;

    // --- Readback initial ---
    reg = 0x0F;
    ret = i2c_master_write_read_device(I2C_PORT, PCF, &reg, 1, &val, 1, 50 / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE("RTC", "PCF8523 NACK @0x0F (CLKOUT): %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGW("RTC", "CLKOUT(0x0F) initial = 0x%02X", val);

    reg = 0x01;
    ret = i2c_master_write_read_device(I2C_PORT, PCF, &reg, 1, &val, 1, 50 / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE("RTC", "PCF8523 NACK @0x01 (Control_2): %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGW("RTC", "CTRL2 (0x01) initial = 0x%02X  [bits: TI_TP|TF|AF|SI|MI|AIE|TIE|CTS]", val);

    // --- 1) Désactive explicitement CLKOUT (broche push-pull) ---
    uint8_t wr_clkout[2] = {0x0F, 0x00};
    ret = i2c_master_write_to_device(I2C_PORT, PCF, wr_clkout, 2, 50 / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE("RTC", "Write 0x0F=0x00 failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // --- 2) Control_2 : TI_TP=1 (pulse), clear TF/AF=0, AIE=0, TIE=1, CTS=0 ---
    // Objectif: ligne INT idle en haute impédance (open-drain), seulement pulses brefs via Timer.
    uint8_t wr_ctrl2[2] = {0x01, (1 << 7) /*TI_TP*/ | (0 << 6) /*TF*/ | (0 << 5) /*AF*/ | (0 << 4) /*SI*/
                                     | (0 << 3) /*MI*/ | (0 << 2) /*AIE*/ | (1 << 1) /*TIE*/ | (0 << 0) /*CTS*/};
    ret = i2c_master_write_to_device(I2C_PORT, PCF, wr_ctrl2, 2, 50 / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE("RTC", "Write CTRL2 failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // --- 3) Readback vérification ---
    reg = 0x0F;
    ret = i2c_master_write_read_device(I2C_PORT, PCF, &reg, 1, &val, 1, 50 / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
        return ret;
    ESP_LOGI("RTC", "CLKOUT(0x0F) after = 0x%02X (attendu 0x00)", val);

    reg = 0x01;
    ret = i2c_master_write_read_device(I2C_PORT, PCF, &reg, 1, &val, 1, 50 / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
        return ret;
    ESP_LOGI("RTC", "CTRL2 (0x01) after  = 0x%02X (attendu TI_TP=1, TIE=1, TF/AF=0)", val);

    // NOTE:  Ici on force un état qui garantit l’open-drain idle.

    return ESP_OK;
}

// Forcer SQW (INT1/CLKOUT) en Hi-Z : désactiver CLKOUT et toutes les IRQ. Sinon 1.1V au noeud
static esp_err_t pcf8523_make_sqw_hi_z(void)
{
    const uint8_t PCF = 0x68;
    esp_err_t ret;
    uint8_t v;

    // 1) 0x0F (Tmr_CLKOUT_ctrl): COF=111 (CLKOUT off), TAC=00 (TimerA off), TBC=0 (TimerB off)
    // Bits: [7]=TAM, [6]=TBM, [5:3]=COF, [2:1]=TAC, [0]=TBC
    // -> valeur 0x38 = 0b00 111 00 0
    uint8_t wr0f[2] = {0x0F, 0x38};
    ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_PORT, PCF, wr0f, 2, 50 / portTICK_PERIOD_MS));

    // 2) 0x01 (Control_2): couper toutes les IRQ + clear flags
    // (lecture efface WTAF ; écriture 0 met AIE/TIE/CTBIE/CTAIE à 0 et nettoie SF/CTAF/CTBF)
    uint8_t wr01[2] = {0x01, 0x00};
    ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_PORT, PCF, wr01, 2, 50 / portTICK_PERIOD_MS));

    // 3) Readback
    uint8_t reg = 0x0F;
    ESP_ERROR_CHECK(i2c_master_write_read_device(I2C_PORT, PCF, &reg, 1, &v, 1, 50 / portTICK_PERIOD_MS));
    ESP_LOGI("RTC", "0x0F=Tmr_CLKOUT_ctrl -> 0x%02X (attendu 0x38)", v);
    reg = 0x01;
    ESP_ERROR_CHECK(i2c_master_write_read_device(I2C_PORT, PCF, &reg, 1, &v, 1, 50 / portTICK_PERIOD_MS));
    ESP_LOGI("RTC", "0x01=Control_2      -> 0x%02X (attendu 0x00)", v);
    return ESP_OK;
}

void app_main(void)
{

    // === CONFIGURATION PWRCONTROL ===
    gpio_config_t pwrcfg = {
        .pin_bit_mask = (1ULL << PWRCONTROL_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&pwrcfg);

    // === I2C INIT ===
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = SCL_GPIO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 50000,
    };
    i2c_param_config(I2C_PORT, &cfg);
    i2c_driver_install(I2C_PORT, cfg.mode, 0, 0, 0);

    // === I2C SCANNER ===
    printf("I2C scanner:\n");
    for (uint8_t addr = 1; addr < 0x7F; ++addr)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 50 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK)
        {
            printf("I2C device found at 0x%02X\n", addr);
        }
        else
        {
            printf("I2C device NO FOUND at 0x%02X\n", addr);
        }
    }

    // === POWER ON PWRCONTROL ===
    gpio_set_level(PWRCONTROL_GPIO, 1);
    ESP_LOGI("GPIO", "PWRCONTROL (IO2) set to HIGH");

    // PASSAGE DE LA SORTIE WAKE DU RTC EN MODE ALARME (PAS CLOCKOUT)

    esp_err_t err = pcf8523_debug_init_irq_timer();
    if (err == ESP_OK)
    {
        ESP_LOGI("RTC", "PCF8523 configuré en IRQ timer open-drain (diagnostic passé).");
    }
    else
    {
        ESP_LOGE("RTC", "PCF8523 init/diag échec: %s (vérifie VDD_RTC, adressage 0x68, câblage INT).", esp_err_to_name(err));
    }

    if (pcf8523_make_sqw_hi_z() == ESP_OK)
    {
        ESP_LOGI("RTC", "SQW forcée Hi-Z.");
    }
    else
    {
        ESP_LOGE("RTC", "Echec sequence Hi-Z.");
    }

    // === TEST E22 (lecture registres) ===
    if (e22_smoke_test() != ESP_OK)
    {
        ESP_LOGW("E22", "Le test E22 a échoué.");
    }
    else
    {
        ESP_LOGI("E22", "Le test E22 a réussi.");
    }
    ESP_ERROR_CHECK(e22_set_tx_power(0 /*MAX*/, true /*permanent*/));
    // xTaskCreatePinnedToCore(e22_burst_tx_task, "e22_burst", 4096, NULL, 5, NULL, tskNO_AFFINITY); // appelle la tâche d’émission périodique, A mettre pour emettre

    // === PCA9536 INIT ===
    esp_err_t ret;
    uint8_t setup[2];

    // REG_POLARITY = 0x00 (pas d'inversion)
    setup[0] = 0x02;
    setup[1] = 0x00;
    ret = i2c_master_write_to_device(I2C_PORT, PCA9536_ADDR, setup, 2, 50 / portTICK_PERIOD_MS);
    ESP_LOGI("I2C", "POLARITY reg write: %s", esp_err_to_name(ret));

    // REG_CONFIG = 0x00 (tous en sortie)
    setup[0] = 0x03;
    setup[1] = 0x00;
    ret = i2c_master_write_to_device(I2C_PORT, PCA9536_ADDR, setup, 2, 50 / portTICK_PERIOD_MS);
    ESP_LOGI("I2C", "CONFIG reg write: %s", esp_err_to_name(ret));

    // === LOOP : Allume/éteint les sorties une par une ===
    while (1)
    {
        for (uint8_t bit = 0; bit < 4; ++bit)
        {
            // Allumer une sortie
            setup[0] = 0x01;        // REG_OUTPUT
            setup[1] = (1u << bit); // GPbit = 1
            ret = i2c_master_write_to_device(I2C_PORT, PCA9536_ADDR, setup, 2, 50 / portTICK_PERIOD_MS);
            ESP_LOGI("PCA9536", "Set GP%d ON (0x%02X), ret = %s", bit, setup[1], esp_err_to_name(ret));
            vTaskDelay(1000 / portTICK_PERIOD_MS);

            // Éteindre toutes les sorties
            setup[1] = 0x00;
            ret = i2c_master_write_to_device(I2C_PORT, PCA9536_ADDR, setup, 2, 50 / portTICK_PERIOD_MS);
            ESP_LOGI("PCA9536", "All OFF, ret = %s", esp_err_to_name(ret));
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}
