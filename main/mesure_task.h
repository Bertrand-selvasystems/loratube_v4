/**
 * @file    mesure_task.h
 * @brief   Mesure au réveil: TSENS + VSENSE (N itérations), push dans 2 queues, puis set eventbit et auto-delete.
 *
 * Usage (dans main) :
 *   mesure_task_cfg_t mcfg = { .n_iter = 5, .delay_ms = 50 };
 *   ESP_ERROR_CHECK(mesure_task_start(&mcfg));
 *
 * Notes :
 * - La config est copiée en statique (1 instance) : pas de malloc.
 * - Pré-requis : system_state_init() et system_queues_init() déjà appelés.
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"

/* Dépendance minimale: eg_state + bit "MEAS_DONE" */
#include "system_state.h"
#include "C3_module.h"

/* Bit attendu (défini idéalement dans system_state.h). */
#ifndef EG_STATE_MEAS_DONE
#define EG_STATE_MEAS_DONE  (1U << 4)
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t n_iter;     /*!< Nombre d'itérations de mesure (0 => 1) */
    uint32_t delay_ms;   /*!< Délai entre itérations (0 => pas de délai) */
} mesure_task_cfg_t;

/**
 * @brief Démarre la task de mesure (auto-delete à la fin).
 *
 * La task :
 * - lit TSENS et VSENSE N fois,
 * - écrit (overwrite) la dernière valeur dans q_temp et q_vbat,
 * - positionne le bit EG_STATE_MEAS_DONE dans eg_state,
 * - se supprime.
 *
 * @param cfg Configuration (peut être sur la stack: elle est copiée).
 * @return esp_err_t
 */
esp_err_t mesure_task_start(const mesure_task_cfg_t *cfg);

#ifdef __cplusplus
}
#endif
