/* Hearing Aid Code 
*/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "board.h"
#include "algorithm_stream.h"
#include "audio_mem.h"
#include "i2s_stream.h"
#include "equalizer.h"

static const char *TAG = "HEARING_AID";

#define I2S_SAMPLE_RATE     44100
#define I2S_CHANNELS        I2S_CHANNEL_FMT_RIGHT_LEFT
#define I2S_BITS            CODEC_ADC_BITS_PER_SAMPLE

#define BUFFER_SIZER    8 * 1024

void app_main()
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_LOGI(TAG, "[1.0] Start codec chip");

    audio_board_handle_t board_handle = (audio_board_handle_t) audio_calloc(1, sizeof(struct audio_board_handle));
    audio_hal_codec_config_t audio_codec_cfg = AUDIO_CODEC_DEFAULT_CONFIG();
    audio_codec_cfg.i2s_iface.samples = AUDIO_HAL_44K_SAMPLES; //AUDIO_HAL_16K_SAMPLES
    board_handle->audio_hal = audio_hal_init(&audio_codec_cfg, &AUDIO_CODEC_ES8311_DEFAULT_HANDLE);
    board_handle->adc_hal = audio_board_adc_init();

    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);
    audio_hal_set_volume(board_handle->audio_hal, 100);

    ESP_LOGI(TAG, "[2.0] Create audio pipeline");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    audio_pipeline_handle_t pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline);

    ESP_LOGI(TAG, "[2.1] Create read stream");
    i2s_stream_cfg_t i2s_cfg_reader = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg_reader.type = AUDIO_STREAM_READER;
    i2s_cfg_reader.i2s_port = I2S_NUM_1;
    i2s_cfg_reader.i2s_config.sample_rate = I2S_SAMPLE_RATE;
    i2s_cfg_reader.out_rb_size = BUFFER_SIZER;
    audio_element_handle_t i2s_stream_reader = i2s_stream_init(&i2s_cfg_reader);

    ESP_LOGI(TAG, "[2.2] Create equalizer");
    equalizer_cfg_t eq_cfg = DEFAULT_EQUALIZER_CONFIG();
    eq_cfg.samplerate=44100;
    eq_cfg.channel=2;
    int set_gain[] = { 0, 0, 7, 9, 10, 9, 12, 17, 17, 0,    0, 0, 14, 0, 11, 17, 21, 23, 22, 0};

    // int set_gain[] = { 0, 0, 9, 11, 10, 9, 12, 19, 17, 0,    0, 0, 16, 4, 11, 17, 21, 23, 22, 0};

    // int set_gain[] = { 0, 0, 10, 11, 10, 9, 12, 18, 17, 0,    0, 0, 15, 6, 11, 17, 21, 23, 22, 0};

    // int set_gain[] = { 0, 0, 10,15,20,15,15,10,10,0,0, 0, 10,15,20,15,15,10,10,0};

    // int set_gain[] = { 25, 13, 13, 9, 10, 9, 12, 17, 17, 0,    25, 13, 13, 0, 11, 17, 21, 23, 22, 0};

    // int set_gain[] = { -13,-13,20,10,-13,-13,-13,20,20,-13,    -13,-13,20,10,-13,-13,-13,20,20,-13};

    // int set_gain[] = { 0, 0, 7, 9, 10, 9, 12, 17, 30, 30,    0, 0, 14, 0, 11, 17, 21, 23, 30, 30};

    // int set_gain[] = { 30, 30, 30, 30, 30, 30, 30, 30, 30, 30,    30, 30, 30, 30, 30, 30, 30, 30, 30, 30};

    // int set_gain[] = { 0,0,0,0,0,0,0,0,0,0,40,40,40,40,40,40,40,40,40,40};

    eq_cfg.set_gain = set_gain; // The size of gain array should be the multiplication of NUMBER_BAND and number channels of audio stream data. The minimum of gain is -13 dB.
    audio_element_handle_t equalizer = equalizer_init(&eq_cfg);

    ESP_LOGI(TAG, "[2.3] Create i2s stream to write data to dac");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    audio_element_handle_t i2s_stream_writer = i2s_stream_init(&i2s_cfg);

    ESP_LOGI(TAG, "[2.4] Register all elements to audio pipeline");
    audio_pipeline_register(pipeline, i2s_stream_reader, "reader");
    audio_pipeline_register(pipeline, i2s_stream_writer, "writer");
    audio_pipeline_register(pipeline, equalizer, "equ");

    ESP_LOGI(TAG, "[2.5] Link it together i2s_stream_reader-->equalizer-->i2s_stream_writer-->[dac]");
    const char *link_rec[3] = {"reader", "equ", "writer"};
    audio_pipeline_link(pipeline, &link_rec[0], 3);

    ESP_LOGI(TAG, "[3.0] Set up event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

    ESP_LOGI(TAG, "[4.0] Start audio_pipeline");

    audio_element_info_t music_info = {0};
    audio_element_getinfo(i2s_stream_writer, &music_info);
    music_info.sample_rates = I2S_SAMPLE_RATE;
    music_info.bits = ALGORITHM_STREAM_DEFAULT_SAMPLE_BIT;
    music_info.channels = 1;

    audio_element_setinfo(i2s_stream_writer, &music_info);

    audio_pipeline_run(pipeline);

    ESP_LOGI(TAG, "[5.0] Listen for all pipeline events");

    while (1) {
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[ * ] Event interface error : %d", ret);
            continue;
        }
    }

    ESP_LOGI(TAG, "[6.0] Stop audio_pipeline");

    audio_pipeline_stop(pipeline);
    audio_pipeline_wait_for_stop(pipeline);
    audio_pipeline_deinit(pipeline);
    audio_pipeline_unregister(pipeline, equalizer);
    audio_element_deinit(equalizer);
    audio_event_iface_destroy(evt);
}