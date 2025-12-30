#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file system_state.h
 * @brief Event groups globaux : eg_state (état stable) et eg_wake (cause/flags de réveil latchés).
 *
 * Règles :
 * - eg_wake : bits latchés du cycle de réveil. SET très tôt (ISR/boot), snapshot + CLEAR par orchestrateur.
 * - eg_state: état courant/stable (modes, actions en cours, erreurs). Peut rester posé longtemps.
 */

// =====================
// Handles (globaux)
// =====================
extern EventGroupHandle_t eg_state;
extern EventGroupHandle_t eg_wake;

// =====================
// eg_wake : wake latches
// =====================

// Causes de réveil
#define EGW_TPL5010      ((EventBits_t)(1u << 0))  // Réveil périodique via TPL5010
#define EGW_RTC          ((EventBits_t)(1u << 1))  // Réveil via alarme/INT RTC
#define EGW_AUX          ((EventBits_t)(1u << 2))  // Réveil via AUX E22 (activité radio)

// Infos latchées utiles au scénario courant (optionnelles)
#define EGW_DAY_CHANGED  ((EventBits_t)(1u << 3))  // Changement de jour détecté
#define EGW_FRAME1       ((EventBits_t)(1u << 4))  // Trame 1 vue pendant ce cycle
#define EGW_FRAME2       ((EventBits_t)(1u << 5))  // Trame 2 vue pendant ce cycle

#define EGW_CAUSE_MASK   (EGW_TPL5010 | EGW_RTC | EGW_AUX)

// =====================
// eg_state : état stable
// =====================

// Actions en cours
#define EGS_RX_ACTIVE        ((EventBits_t)(1u << 0))  // Radio en réception (activité en cours)
#define EGS_TX_ACTIVE        ((EventBits_t)(1u << 1))  // Radio en émission (activité en cours)

// Acquisitions / données valides
#define EGS_TEMP_VALID       ((EventBits_t)(1u << 2))  // Température mesurée OK (valeur dispo)
#define EGS_VBAT_VALID       ((EventBits_t)(1u << 3))  // VBAT mesurée OK (valeur dispo)
#define EGS_DATE_VALID       ((EventBits_t)(1u << 4))  // VBAT mesurée OK (valeur dispo)

// États actionneurs (debug/cohérence)
#define EGS_LED_GREEN_ON     ((EventBits_t)(1u << 5))
#define EGS_LED_RED_ON       ((EventBits_t)(1u << 6))
#define EGS_BUCK_FORCED_ON   ((EventBits_t)(1u << 7))  // Buck forcé ON (non ECO)
#define EGS_E22_POWERED      ((EventBits_t)(1u << 8))  // Alim E22 active

// Erreurs latched
#define EGS_ERR_FRAM         ((EventBits_t)(1u << 9))
#define EGS_ERR_RTC          ((EventBits_t)(1u << 10))
#define EGS_ERR_PCA          ((EventBits_t)(1u << 11))
#define EGS_ERR_E22          ((EventBits_t)(1u << 12))
#define EGS_ERR_LOW_VBAT     ((EventBits_t)(1u << 13))
#define EGS_ERR_DATE         ((EventBits_t)(1u << 14))

// =====================
// Init / helpers
// =====================

/**
 * @brief Crée eg_state et eg_wake.
 * @return ESP_OK si OK, sinon ESP_ERR_NO_MEM ou ESP_ERR_INVALID_STATE.
 */
esp_err_t system_state_init(void);

/**
 * @brief Détruit eg_state et eg_wake (si tu en as besoin en tests).
 */
void system_state_deinit(void);

// Helpers "state"
static inline void state_set(EventBits_t bits)   { (void)xEventGroupSetBits(eg_state, bits); }
static inline void state_clear(EventBits_t bits) { (void)xEventGroupClearBits(eg_state, bits); }
static inline EventBits_t state_get(void)        { return xEventGroupGetBits(eg_state); }

// Helpers "wake"
static inline void wake_set(EventBits_t bits)    { (void)xEventGroupSetBits(eg_wake, bits); }
static inline void wake_clear(EventBits_t bits)  { (void)xEventGroupClearBits(eg_wake, bits); }
static inline EventBits_t wake_get(void)         { return xEventGroupGetBits(eg_wake); }

#ifdef __cplusplus
}
#endif
