#include "system_state.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

static const char *TAG = "SYS_STATE";

EventGroupHandle_t eg_state = NULL;
EventGroupHandle_t eg_wake  = NULL;

esp_err_t system_state_init(void)
{
    if (eg_state || eg_wake) {
        // déjà initialisé (ou init partielle) -> refuse pour éviter états bizarres
        return ESP_ERR_INVALID_STATE;
    }

    eg_state = xEventGroupCreate();
    eg_wake  = xEventGroupCreate();

    if (!eg_state || !eg_wake) {
        if (eg_state) { vEventGroupDelete(eg_state); eg_state = NULL; }
        if (eg_wake)  { vEventGroupDelete(eg_wake);  eg_wake  = NULL; }
        ESP_LOGE(TAG, "xEventGroupCreate failed");
        return ESP_ERR_NO_MEM;
    }

    // Valeurs initiales possibles (optionnel) :
    // - rien par défaut.
    // - tu peux aussi set EGS_E22_POWERED quand tu l'actives réellement, pas ici.

    ESP_LOGI(TAG, "event groups ready (eg_state, eg_wake)");
    return ESP_OK;
}

void system_state_deinit(void)
{
    if (eg_state) {
        vEventGroupDelete(eg_state);
        eg_state = NULL;
    }
    if (eg_wake) {
        vEventGroupDelete(eg_wake);
        eg_wake = NULL;
    }
}
