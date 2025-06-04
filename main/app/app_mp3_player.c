#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "bsp_board_extra.h"
#include "app_mp3_player.h"
#include "mmap_generate_audio.h"

static const char *TAG = "app_mp3_player";

static esp_codec_dev_handle_t spk_codec_dev = NULL;
static bool is_playing = false;
static bool is_paused = false;
static int current_position = 0;
static int total_duration = 0;
static const char *current_file = NULL;

extern mmap_assets_handle_t asset_audio;

esp_err_t app_mp3_player_init(void)
{
    // 初始化音频编解码器
    spk_codec_dev = bsp_extra_audio_codec_speaker_init();
    if (!spk_codec_dev) {
        ESP_LOGE(TAG, "Failed to initialize audio codec");
        return ESP_FAIL;
    }

    // 设置音频参数
    esp_codec_dev_sample_info_t fs = {
        .sample_rate = 44100,
        .channel = 2,
        .bits_per_sample = 16,
    };
    esp_codec_dev_open(spk_codec_dev, &fs);
    esp_codec_dev_set_out_vol(spk_codec_dev, 60);
    
    return ESP_OK;
}

esp_err_t app_mp3_player_play(const char* filename)
{
    if (spk_codec_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // 获取音频数据
    int file_index = -1;
    if (strcmp(filename, "echo.wav") == 0) {
        file_index = MMAP_AUDIO_ECHO_WAV;
    } else if (strcmp(filename, "echo1.wav") == 0) {
        file_index = MMAP_AUDIO_ECHO1_WAV;
    } else if (strcmp(filename, "muyu.wav") == 0) {
        file_index = MMAP_AUDIO_MUYU_WAV;
    } else if (strcmp(filename, "jntm.wav") == 0) {
        file_index = MMAP_AUDIO_JNTM_WAV;
    }
    
    if (file_index == -1) {
        ESP_LOGE(TAG, "File not found: %s", filename);
        return ESP_ERR_NOT_FOUND;
    }

    void *audio_data = (void *)mmap_assets_get_mem(asset_audio, file_index);
    if (audio_data == NULL) {
        ESP_LOGE(TAG, "Failed to get audio data");
        return ESP_FAIL;
    }

    uint32_t audio_len = mmap_assets_get_size(asset_audio, file_index);
    if (audio_len == 0) {
        ESP_LOGE(TAG, "Invalid audio length");
        return ESP_FAIL;
    }

    // 播放音频
    int ret = esp_codec_dev_write(spk_codec_dev, audio_data, audio_len);
    if (ret < 0) {
        ESP_LOGE(TAG, "Failed to play audio");
        return ESP_FAIL;
    }

    is_playing = true;
    is_paused = false;
    current_file = filename;
    total_duration = audio_len / (44100 * 4); // 44.1kHz, 16位, 立体声
    current_position = 0;

    return ESP_OK;
}

esp_err_t app_mp3_player_pause(void)
{
    if (!is_playing || is_paused) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_codec_dev_set_out_mute(spk_codec_dev, true);
    is_paused = true;
    return ESP_OK;
}

esp_err_t app_mp3_player_resume(void)
{
    if (!is_playing || !is_paused) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_codec_dev_set_out_mute(spk_codec_dev, false);
    is_paused = false;
    return ESP_OK;
}

bool app_mp3_player_is_playing(void)
{
    return is_playing && !is_paused;
}

int app_mp3_player_get_progress(void)
{
    if (!is_playing || total_duration == 0) {
        return 0;
    }
    return (current_position * 100) / total_duration;
}

int app_mp3_player_get_position(void)
{
    return current_position;
}

int app_mp3_player_get_duration(void)
{
    return total_duration;
}
