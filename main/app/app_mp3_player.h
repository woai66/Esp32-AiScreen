#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// 初始化MP3播放器
esp_err_t app_mp3_player_init(void);

// 播放指定文件
esp_err_t app_mp3_player_play(const char* filename);

// 暂停播放
esp_err_t app_mp3_player_pause(void);

// 继续播放
esp_err_t app_mp3_player_resume(void);

// 获取播放状态
bool app_mp3_player_is_playing(void);

// 获取播放进度(0-100)
int app_mp3_player_get_progress(void);

// 获取当前播放位置(秒)
int app_mp3_player_get_position(void);

// 获取音频总时长(秒)
int app_mp3_player_get_duration(void);
esp_err_t app_mp3_player_stop(void);

#ifdef __cplusplus
}
#endif
