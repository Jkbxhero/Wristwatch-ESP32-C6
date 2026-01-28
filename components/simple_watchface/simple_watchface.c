/**
 * @file simple_watchface.c
 * @brief Simple text-based watchface implementation
 */

#include "simple_watchface.h"
#include <stdio.h>
#include <stdlib.h>

// Display dimensions (from BSP - 410x502 for this watch)
#define DISPLAY_WIDTH   410
#define DISPLAY_HEIGHT  502
#define CENTER_X        (DISPLAY_WIDTH / 2)
#define CENTER_Y        (DISPLAY_HEIGHT / 2)

// Colors
#define COLOR_BACKGROUND    lv_color_hex(0x000000)  // Black
#define COLOR_TIME_TEXT     lv_color_hex(0xFFFFFF)  // White
#define COLOR_DATE_TEXT     lv_color_hex(0x888888)  // Gray

simple_watchface_t* simple_watchface_create(lv_obj_t *parent)
{
    simple_watchface_t *wf = (simple_watchface_t *)malloc(sizeof(simple_watchface_t));
    if (!wf) return NULL;

    // Store reference to screen
    wf->screen = parent;

    // Set background color
    lv_obj_set_style_bg_color(parent, COLOR_BACKGROUND, LV_PART_MAIN);

    // Create time label (large, centered)
    wf->time_label = lv_label_create(parent);
    lv_label_set_text(wf->time_label, "00:00:00");
    lv_obj_set_style_text_color(wf->time_label, COLOR_TIME_TEXT, 0);
    lv_obj_set_style_text_font(wf->time_label, &lv_font_montserrat_48, 0);
    lv_obj_align(wf->time_label, LV_ALIGN_CENTER, 0, -30);

    // Create date label (smaller, below time)
    wf->date_label = lv_label_create(parent);
    lv_label_set_text(wf->date_label, "--- --");
    lv_obj_set_style_text_color(wf->date_label, COLOR_DATE_TEXT, 0);
    lv_obj_set_style_text_font(wf->date_label, &lv_font_montserrat_24, 0);
    lv_obj_align(wf->date_label, LV_ALIGN_CENTER, 0, 40);

    // AM/PM label (optional, for 12-hour format)
    wf->ampm_label = lv_label_create(parent);
    lv_label_set_text(wf->ampm_label, "");
    lv_obj_set_style_text_color(wf->ampm_label, COLOR_DATE_TEXT, 0);
    lv_obj_set_style_text_font(wf->ampm_label, &lv_font_montserrat_20, 0);
    lv_obj_align(wf->ampm_label, LV_ALIGN_CENTER, 100, -40);

    return wf;
}

void simple_watchface_set_time(simple_watchface_t *wf, int hour, int minute, int second)
{
    if (!wf || !wf->time_label) return;

    char time_str[12];
    snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", hour, minute, second);
    lv_label_set_text(wf->time_label, time_str);
}

void simple_watchface_set_date(simple_watchface_t *wf, const char *day_name, int day_num)
{
    if (!wf || !wf->date_label) return;

    char date_str[12];
    snprintf(date_str, sizeof(date_str), "%s %02d", day_name ? day_name : "---", day_num);
    lv_label_set_text(wf->date_label, date_str);
}

void simple_watchface_destroy(simple_watchface_t *wf)
{
    if (!wf) return;

    // Delete LVGL objects explicitly (parent screen is reused, not deleted)
    if (wf->time_label) {
        lv_obj_del(wf->time_label);
        wf->time_label = NULL;
    }
    if (wf->date_label) {
        lv_obj_del(wf->date_label);
        wf->date_label = NULL;
    }
    if (wf->ampm_label) {
        lv_obj_del(wf->ampm_label);
        wf->ampm_label = NULL;
    }

    // Free our structure
    free(wf);
}

// Stub - simple watchface doesn't display accelerometer data
void simple_watchface_set_accel(simple_watchface_t *wf, float x, float y, float z)
{
    // No-op: Simple watchface doesn't have accel sub-dials
    // This stub exists for compatibility with IMU task
    (void)wf;
    (void)x;
    (void)y;
    (void)z;
}
