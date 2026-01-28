#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "esp_dsp.h"
#include "bsp_board_extra.h"
#include "Spec_Analyzer.h"

#define TAG "audio_fft"

// FFT parameters
#define N_SAMPLES           1024
#define SAMPLE_RATE         16000
#define CHANNELS            2
#define STRIPE_COUNT        64

// Display area
#define CANVAS_WIDTH        240
#define CANVAS_HEIGHT       120

// Audio and FFT buffers
__attribute__((aligned(16))) static int16_t raw_data[N_SAMPLES * CHANNELS];
__attribute__((aligned(16))) static float audio_buffer[N_SAMPLES];
__attribute__((aligned(16))) static float wind[N_SAMPLES];
__attribute__((aligned(16))) static float fft_buffer[N_SAMPLES * 2];
__attribute__((aligned(16))) static float spectrum[N_SAMPLES / 2];

static float display_spectrum[STRIPE_COUNT];
static float peak[STRIPE_COUNT];

// Module state
static TaskHandle_t audio_fft_task_handle = NULL;
static lv_timer_t *spectrum_timer = NULL;
static lv_obj_t *spectrum_canvas = NULL;
static bool analyzer_running = false;

// Static draw buffer for canvas
LV_DRAW_BUF_DEFINE_STATIC(spectrum_draw_buf, CANVAS_WIDTH, CANVAS_HEIGHT, LV_COLOR_FORMAT_RGB565);

/* ------------------ Audio FFT Task ------------------ */
static void audio_fft_task(void *pvParameters)
{
    esp_err_t ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "FFT init failed: %d", ret);
        vTaskDelete(NULL);
        return;
    }

    dsps_wind_hann_f32(wind, N_SAMPLES);
    ESP_LOGI(TAG, "FFT and window initialized");

    ESP_LOGI(TAG, "Initializing audio codec, heap free: %u",
             heap_caps_get_free_size(MALLOC_CAP_8BIT));

    if (bsp_extra_codec_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "Audio codec init failed");
        analyzer_running = false;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Audio codec initialized, heap free: %u",
             heap_caps_get_free_size(MALLOC_CAP_8BIT));

    TickType_t last_wake_time = xTaskGetTickCount();
    size_t bytes_read;

    // Small delay for codec to stabilize before first read
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Starting audio capture loop");

    while (analyzer_running)
    {
        ret = bsp_extra_i2s_read(raw_data, N_SAMPLES * CHANNELS * sizeof(int16_t), &bytes_read, pdMS_TO_TICKS(100));
        if (ret != ESP_OK || bytes_read != N_SAMPLES * CHANNELS * sizeof(int16_t))
        {
            if (analyzer_running) {
                ESP_LOGW(TAG, "I2S read error: %d, bytes: %d", ret, bytes_read);
            }
            continue;
        }

        // Merge left and right channels and normalize
        for (int i = 0; i < N_SAMPLES; i++)
        {
            int16_t left = raw_data[i * CHANNELS];
            int16_t right = raw_data[i * CHANNELS + 1];
            audio_buffer[i] = (left + right) / (2.0f * 32768.0f);
        }

        dsps_mul_f32(audio_buffer, wind, audio_buffer, N_SAMPLES, 1, 1, 1);

        // Fill FFT input buffer
        for (int i = 0; i < N_SAMPLES; i++)
        {
            fft_buffer[2 * i] = audio_buffer[i];
            fft_buffer[2 * i + 1] = 0;
        }

        dsps_fft2r_fc32(fft_buffer, N_SAMPLES);
        dsps_bit_rev_fc32(fft_buffer, N_SAMPLES);

        // Calculate magnitude spectrum (dB)
        for (int i = 0; i < N_SAMPLES / 2; i++)
        {
            float real = fft_buffer[2 * i];
            float imag = fft_buffer[2 * i + 1];
            float magnitude = sqrtf(real * real + imag * imag);
            spectrum[i] = 20 * log10f(magnitude / (N_SAMPLES / 2) + 1e-9);
        }

        // Map to display bandwidth
        for (int i = 0; i < STRIPE_COUNT; i++)
        {
            int fft_idx = i * (N_SAMPLES / 2) / STRIPE_COUNT;
            display_spectrum[i] = fmaxf(-90.0f, fminf(0.0f, spectrum[fft_idx]));
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
    }

    ESP_LOGI(TAG, "Audio FFT task exiting");
    audio_fft_task_handle = NULL;
    vTaskDelete(NULL);
}

/* ------------------ Display Update Timer ------------------ */
static void timer_cb(lv_timer_t *timer)
{
    // Double-check state before proceeding
    if (!analyzer_running || !spectrum_canvas) return;

    // Acquire display lock to prevent race conditions during screen transitions
    bsp_display_lock(0);

    // Re-check state after acquiring lock (state could have changed while waiting)
    if (!analyzer_running || !spectrum_canvas) {
        bsp_display_unlock();
        return;
    }

    lv_layer_t layer;
    lv_canvas_init_layer(spectrum_canvas, &layer);
    lv_canvas_fill_bg(spectrum_canvas, lv_color_black(), LV_OPA_COVER);

    const float total_width = CANVAS_WIDTH;
    const float total_gap = 2.0f * STRIPE_COUNT;
    const float stripe_width_f = (total_width - total_gap) / STRIPE_COUNT;
    const int center_y = CANVAS_HEIGHT / 2;

    for (int i = 0; i < STRIPE_COUNT; i++) {
        float db = display_spectrum[i];
        float db_min = -90.0f, db_max = 0.0f;
        float norm = (db - db_min) / (db_max - db_min);
        norm = fmaxf(0.0f, fminf(1.0f, norm));
        norm = sqrtf(norm);

        int bar_height = (int)(norm * (CANVAS_HEIGHT / 2));

        if (peak[i] < bar_height) peak[i] = bar_height;
        else {
            peak[i] -= 2;
            if (peak[i] < 0) peak[i] = 0;
        }

        float hue_step = 270.0f / STRIPE_COUNT;
        uint16_t hue = (uint16_t)(i * hue_step);
        lv_color_t color = lv_color_hsv_to_rgb(hue, 100, 100);

        lv_draw_rect_dsc_t rect_dsc;
        lv_draw_rect_dsc_init(&rect_dsc);
        rect_dsc.bg_color = color;
        rect_dsc.bg_opa = LV_OPA_COVER;

        int x_start = (int)roundf(i * (stripe_width_f + 2));
        int x_end   = (int)roundf(x_start + stripe_width_f);

        lv_area_t bar_area = {
            .x1 = x_start,
            .y1 = center_y - bar_height,
            .x2 = x_end,
            .y2 = center_y + bar_height
        };
        lv_draw_rect(&layer, &rect_dsc, &bar_area);

        // Peak particles at top and bottom
        int peak_y_top = center_y - (int)peak[i] - 2;
        int peak_y_bot = center_y + (int)peak[i];

        lv_area_t particle_area_top = {
            .x1 = x_start,
            .y1 = peak_y_top,
            .x2 = x_end,
            .y2 = peak_y_top + 2
        };
        lv_draw_rect(&layer, &rect_dsc, &particle_area_top);

        lv_area_t particle_area_bot = {
            .x1 = x_start,
            .y1 = peak_y_bot,
            .x2 = x_end,
            .y2 = peak_y_bot + 2
        };
        lv_draw_rect(&layer, &rect_dsc, &particle_area_bot);
    }

    lv_canvas_finish_layer(spectrum_canvas, &layer);

    bsp_display_unlock();
}

/* ------------------ Public API ------------------ */
#define MIN_HEAP_FOR_AUDIO  50000  // Audio codec + I2S needs ~40KB

esp_err_t spec_analyzer_start(lv_obj_t *parent)
{
    if (analyzer_running) {
        ESP_LOGW(TAG, "Spectrum analyzer already running");
        return ESP_ERR_INVALID_STATE;
    }

    // Check heap before starting - audio codec needs significant RAM
    size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "Starting spectrum analyzer, free heap: %u bytes", free_heap);

    if (free_heap < MIN_HEAP_FOR_AUDIO) {
        ESP_LOGE(TAG, "Insufficient heap for audio! Need %d, have %u",
                 MIN_HEAP_FOR_AUDIO, free_heap);
        return ESP_ERR_NO_MEM;
    }

    // Reset peak values
    memset(peak, 0, sizeof(peak));
    memset(display_spectrum, 0, sizeof(display_spectrum));

    // Initialize draw buffer
    LV_DRAW_BUF_INIT_STATIC(spectrum_draw_buf);

    // Create canvas on parent
    spectrum_canvas = lv_canvas_create(parent);
    if (!spectrum_canvas) {
        ESP_LOGE(TAG, "Failed to create canvas");
        return ESP_ERR_NO_MEM;
    }
    lv_obj_set_size(spectrum_canvas, CANVAS_WIDTH, CANVAS_HEIGHT);
    lv_obj_align(spectrum_canvas, LV_ALIGN_CENTER, 0, 0);
    lv_canvas_set_draw_buf(spectrum_canvas, &spectrum_draw_buf);

    // Create timer for display updates (~30 fps)
    spectrum_timer = lv_timer_create(timer_cb, 33, NULL);
    if (!spectrum_timer) {
        ESP_LOGE(TAG, "Failed to create timer");
        lv_obj_del(spectrum_canvas);
        spectrum_canvas = NULL;
        return ESP_ERR_NO_MEM;
    }

    analyzer_running = true;

    // Create FFT task
    BaseType_t ret = xTaskCreate(audio_fft_task, "audio_fft", 6 * 1024, NULL, 5, &audio_fft_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create FFT task");
        analyzer_running = false;
        lv_timer_del(spectrum_timer);
        spectrum_timer = NULL;
        lv_obj_del(spectrum_canvas);
        spectrum_canvas = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Spectrum analyzer started");
    return ESP_OK;
}

void spec_analyzer_stop(void)
{
    if (!analyzer_running) {
        return;
    }

    ESP_LOGI(TAG, "Stopping spectrum analyzer");

    // Signal task to stop first
    analyzer_running = false;

    // Wait for FFT task to exit (it checks analyzer_running in its loop)
    if (audio_fft_task_handle != NULL) {
        // Give task time to notice the flag and exit cleanly
        // Increased timeout to allow I2S reads to complete
        vTaskDelay(pdMS_TO_TICKS(300));

        // If still running, force delete
        if (audio_fft_task_handle != NULL) {
            vTaskDelete(audio_fft_task_handle);
            audio_fft_task_handle = NULL;
        }
    }

    // Acquire display lock before modifying LVGL objects
    // This ensures timer callback is not running (it also holds the lock)
    bsp_display_lock(0);

    // Clear canvas pointer first - timer callback rechecks this after acquiring lock
    spectrum_canvas = NULL;

    // Now delete timer safely
    if (spectrum_timer) {
        lv_timer_del(spectrum_timer);
        spectrum_timer = NULL;
    }

    bsp_display_unlock();

    // CRITICAL: Stop and close the audio codec to free resources
    // This was missing and caused memory leak on each audio screen visit
    bsp_extra_codec_dev_stop();
    ESP_LOGI(TAG, "Audio codec stopped");

    ESP_LOGI(TAG, "Spectrum analyzer stopped");
}
