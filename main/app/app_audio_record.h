/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "esp_afe_sr_models.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    AUDIO_WAKENET_END = 0,
    AUDIO_WAKENET_START,
    AUDIO_VAD_END,
    AUDIO_VAD_START,
    AUDIO_VAD_WAIT,
    AUDIO_PLAY_MUYU,
    AUDIO_PLAY_ECHO1,
    AUDIO_PLAY_PAUSE,
    AUDIO_PLAY_RESUME,
} audio_record_state_t;


esp_err_t app_sr_start(void);

void app_sr_paly_muyu();
void app_sr_play_echo1();
void app_sr_play_pause();
void app_sr_play_resume();

#ifdef __cplusplus
}
#endif