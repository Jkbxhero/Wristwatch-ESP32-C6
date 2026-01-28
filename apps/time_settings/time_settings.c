/**
 * @file time_settings.c
 * @brief Time settings app implementation
 */

#include "time_settings.h"
#include <stdlib.h>
#include <stdio.h>

// Store callbacks (static since only one instance at a time)
static time_settings_exit_cb_t s_exit_cb = NULL;
static time_settings_mode_cb_t s_mode_cb = NULL;
static time_settings_refresh_cb_t s_refresh_cb = NULL;
static time_settings_set_cb_t s_set_cb = NULL;

// Store pointer for callbacks that need it
static time_settings_t *s_current_ts = NULL;

// Forward declarations
static void return_btn_cb(lv_event_t *e);
static void switch_cb(lv_event_t *e);
static void refresh_btn_cb(lv_event_t *e);
static void set_btn_cb(lv_event_t *e);
static void update_mode_visibility(time_settings_t *ts, bool is_manual);

// Generate roller options string for hours (1-24)
static const char *hour_options =
    "1\n2\n3\n4\n5\n6\n7\n8\n9\n10\n11\n12\n"
    "13\n14\n15\n16\n17\n18\n19\n20\n21\n22\n23\n24";

// Generate roller options string for minutes (00-59)
static const char *minute_options =
    "00\n01\n02\n03\n04\n05\n06\n07\n08\n09\n"
    "10\n11\n12\n13\n14\n15\n16\n17\n18\n19\n"
    "20\n21\n22\n23\n24\n25\n26\n27\n28\n29\n"
    "30\n31\n32\n33\n34\n35\n36\n37\n38\n39\n"
    "40\n41\n42\n43\n44\n45\n46\n47\n48\n49\n"
    "50\n51\n52\n53\n54\n55\n56\n57\n58\n59";

// Generate roller options string for months (1-12)
static const char *month_options = "1\n2\n3\n4\n5\n6\n7\n8\n9\n10\n11\n12";

// Generate roller options string for days (1-31)
static const char *day_options =
    "1\n2\n3\n4\n5\n6\n7\n8\n9\n10\n"
    "11\n12\n13\n14\n15\n16\n17\n18\n19\n20\n"
    "21\n22\n23\n24\n25\n26\n27\n28\n29\n30\n31";

// Generate roller options string for years (2025-2050)
static const char *year_options =
    "2025\n2026\n2027\n2028\n2029\n2030\n2031\n2032\n2033\n2034\n"
    "2035\n2036\n2037\n2038\n2039\n2040\n2041\n2042\n2043\n2044\n"
    "2045\n2046\n2047\n2048\n2049\n2050";

time_settings_t* time_settings_create(lv_obj_t *parent,
                                       time_settings_exit_cb_t exit_cb,
                                       time_settings_mode_cb_t mode_cb,
                                       time_settings_refresh_cb_t refresh_cb,
                                       time_settings_set_cb_t set_cb)
{
    time_settings_t *ts = (time_settings_t *)malloc(sizeof(time_settings_t));
    if (!ts) {
        return NULL;
    }

    // Store callbacks
    s_exit_cb = exit_cb;
    s_mode_cb = mode_cb;
    s_refresh_cb = refresh_cb;
    s_set_cb = set_cb;
    s_current_ts = ts;

    // Main container (black background, full screen)
    ts->container = lv_obj_create(parent);
    lv_obj_set_size(ts->container, 410, 502);
    lv_obj_center(ts->container);
    lv_obj_set_style_bg_color(ts->container, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(ts->container, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(ts->container, 0, 0);
    lv_obj_set_style_radius(ts->container, 0, 0);
    lv_obj_remove_flag(ts->container, LV_OBJ_FLAG_SCROLLABLE);

    // Title label - "Time Settings"
    ts->title_label = lv_label_create(ts->container);
    lv_label_set_text(ts->title_label, "Time Settings");
    lv_obj_set_style_text_color(ts->title_label, lv_color_hex(0x88aacc), 0);
    lv_obj_set_style_text_font(ts->title_label, &lv_font_montserrat_24, 0);
    lv_obj_align(ts->title_label, LV_ALIGN_TOP_MID, 0, 30);

    // Create a container for the switch row (auto - switch - manual)
    lv_obj_t *switch_row = lv_obj_create(ts->container);
    lv_obj_set_size(switch_row, 280, 50);
    lv_obj_align(switch_row, LV_ALIGN_TOP_MID, 0, 70);
    lv_obj_set_style_bg_opa(switch_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(switch_row, 0, 0);
    lv_obj_remove_flag(switch_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(switch_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(switch_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // "Auto" label (left side - off position)
    ts->auto_label = lv_label_create(switch_row);
    lv_label_set_text(ts->auto_label, "Auto");
    lv_obj_set_style_text_color(ts->auto_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(ts->auto_label, &lv_font_montserrat_18, 0);

    // Mode switch
    ts->mode_switch = lv_switch_create(switch_row);
    lv_obj_set_size(ts->mode_switch, 70, 35);
    lv_obj_set_style_bg_color(ts->mode_switch, lv_color_hex(0x1a3a5c), 0);
    lv_obj_set_style_bg_color(ts->mode_switch, lv_color_hex(0x2a6a9c), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(ts->mode_switch, lv_color_hex(0x0a1628), LV_PART_INDICATOR);
    lv_obj_add_event_cb(ts->mode_switch, switch_cb, LV_EVENT_VALUE_CHANGED, ts);

    // "Manual" label (right side - on position)
    ts->manual_label = lv_label_create(switch_row);
    lv_label_set_text(ts->manual_label, "Manual");
    lv_obj_set_style_text_color(ts->manual_label, lv_color_hex(0x666666), 0);
    lv_obj_set_style_text_font(ts->manual_label, &lv_font_montserrat_18, 0);

    // ===== AUTO MODE ELEMENTS =====

    // Refresh button (auto mode only) - 150 wide x 75 tall
    ts->refresh_btn = lv_btn_create(ts->container);
    lv_obj_set_size(ts->refresh_btn, 150, 75);
    lv_obj_align(ts->refresh_btn, LV_ALIGN_CENTER, 0, 20);
    lv_obj_set_style_bg_color(ts->refresh_btn, lv_color_hex(0x1a3a5c), 0);
    lv_obj_set_style_bg_color(ts->refresh_btn, lv_color_hex(0x2a5a8c), LV_STATE_PRESSED);
    lv_obj_set_style_radius(ts->refresh_btn, 10, 0);
    lv_obj_add_event_cb(ts->refresh_btn, refresh_btn_cb, LV_EVENT_CLICKED, ts);

    lv_obj_t *refresh_label = lv_label_create(ts->refresh_btn);
    lv_label_set_text(refresh_label, "Refresh");
    lv_obj_set_style_text_color(refresh_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(refresh_label, &lv_font_montserrat_20, 0);
    lv_obj_center(refresh_label);

    // ===== MANUAL MODE ELEMENTS =====

    // Time row labels
    ts->hour_label = lv_label_create(ts->container);
    lv_label_set_text(ts->hour_label, "Hour");
    lv_obj_set_style_text_color(ts->hour_label, lv_color_hex(0x88aacc), 0);
    lv_obj_set_style_text_font(ts->hour_label, &lv_font_montserrat_14, 0);
    lv_obj_align(ts->hour_label, LV_ALIGN_TOP_LEFT, 95, 125);

    ts->minute_label = lv_label_create(ts->container);
    lv_label_set_text(ts->minute_label, "Min");
    lv_obj_set_style_text_color(ts->minute_label, lv_color_hex(0x88aacc), 0);
    lv_obj_set_style_text_font(ts->minute_label, &lv_font_montserrat_14, 0);
    lv_obj_align(ts->minute_label, LV_ALIGN_TOP_LEFT, 220, 125);

    // Hour roller (1-24)
    ts->hour_roller = lv_roller_create(ts->container);
    lv_roller_set_options(ts->hour_roller, hour_options, LV_ROLLER_MODE_NORMAL);
    lv_roller_set_visible_row_count(ts->hour_roller, 3);
    lv_obj_set_width(ts->hour_roller, 80);
    lv_obj_align(ts->hour_roller, LV_ALIGN_TOP_LEFT, 70, 145);
    lv_obj_set_style_bg_color(ts->hour_roller, lv_color_hex(0x1a3a5c), 0);
    lv_obj_set_style_bg_color(ts->hour_roller, lv_color_hex(0x2a5a8c), LV_PART_SELECTED);
    lv_obj_set_style_text_color(ts->hour_roller, lv_color_white(), 0);
    lv_obj_set_style_text_font(ts->hour_roller, &lv_font_montserrat_20, 0);
    lv_obj_set_style_border_width(ts->hour_roller, 0, 0);

    // Minute roller (0-59)
    ts->minute_roller = lv_roller_create(ts->container);
    lv_roller_set_options(ts->minute_roller, minute_options, LV_ROLLER_MODE_NORMAL);
    lv_roller_set_visible_row_count(ts->minute_roller, 3);
    lv_obj_set_width(ts->minute_roller, 80);
    lv_obj_align(ts->minute_roller, LV_ALIGN_TOP_LEFT, 195, 145);
    lv_obj_set_style_bg_color(ts->minute_roller, lv_color_hex(0x1a3a5c), 0);
    lv_obj_set_style_bg_color(ts->minute_roller, lv_color_hex(0x2a5a8c), LV_PART_SELECTED);
    lv_obj_set_style_text_color(ts->minute_roller, lv_color_white(), 0);
    lv_obj_set_style_text_font(ts->minute_roller, &lv_font_montserrat_20, 0);
    lv_obj_set_style_border_width(ts->minute_roller, 0, 0);

    // Date row - Month, Day, Year rollers
    // Month roller (1-12)
    ts->month_roller = lv_roller_create(ts->container);
    lv_roller_set_options(ts->month_roller, month_options, LV_ROLLER_MODE_NORMAL);
    lv_roller_set_visible_row_count(ts->month_roller, 3);
    lv_obj_set_width(ts->month_roller, 60);
    lv_obj_align(ts->month_roller, LV_ALIGN_TOP_LEFT, 50, 245);
    lv_obj_set_style_bg_color(ts->month_roller, lv_color_hex(0x1a3a5c), 0);
    lv_obj_set_style_bg_color(ts->month_roller, lv_color_hex(0x2a5a8c), LV_PART_SELECTED);
    lv_obj_set_style_text_color(ts->month_roller, lv_color_white(), 0);
    lv_obj_set_style_text_font(ts->month_roller, &lv_font_montserrat_18, 0);
    lv_obj_set_style_border_width(ts->month_roller, 0, 0);

    // Day roller (1-31)
    ts->day_roller = lv_roller_create(ts->container);
    lv_roller_set_options(ts->day_roller, day_options, LV_ROLLER_MODE_NORMAL);
    lv_roller_set_visible_row_count(ts->day_roller, 3);
    lv_obj_set_width(ts->day_roller, 60);
    lv_obj_align(ts->day_roller, LV_ALIGN_TOP_LEFT, 140, 245);
    lv_obj_set_style_bg_color(ts->day_roller, lv_color_hex(0x1a3a5c), 0);
    lv_obj_set_style_bg_color(ts->day_roller, lv_color_hex(0x2a5a8c), LV_PART_SELECTED);
    lv_obj_set_style_text_color(ts->day_roller, lv_color_white(), 0);
    lv_obj_set_style_text_font(ts->day_roller, &lv_font_montserrat_18, 0);
    lv_obj_set_style_border_width(ts->day_roller, 0, 0);

    // Year roller (2025-2050)
    ts->year_roller = lv_roller_create(ts->container);
    lv_roller_set_options(ts->year_roller, year_options, LV_ROLLER_MODE_NORMAL);
    lv_roller_set_visible_row_count(ts->year_roller, 3);
    lv_obj_set_width(ts->year_roller, 90);
    lv_obj_align(ts->year_roller, LV_ALIGN_TOP_LEFT, 230, 245);
    lv_obj_set_style_bg_color(ts->year_roller, lv_color_hex(0x1a3a5c), 0);
    lv_obj_set_style_bg_color(ts->year_roller, lv_color_hex(0x2a5a8c), LV_PART_SELECTED);
    lv_obj_set_style_text_color(ts->year_roller, lv_color_white(), 0);
    lv_obj_set_style_text_font(ts->year_roller, &lv_font_montserrat_18, 0);
    lv_obj_set_style_border_width(ts->year_roller, 0, 0);

    // Set button (manual mode only)
    ts->set_btn = lv_btn_create(ts->container);
    lv_obj_set_size(ts->set_btn, 120, 50);
    lv_obj_align(ts->set_btn, LV_ALIGN_BOTTOM_LEFT, 25, -25);
    lv_obj_set_style_bg_color(ts->set_btn, lv_color_hex(0x2a6a3c), 0);  // Green tint
    lv_obj_set_style_bg_color(ts->set_btn, lv_color_hex(0x3a8a4c), LV_STATE_PRESSED);
    lv_obj_set_style_radius(ts->set_btn, 10, 0);
    lv_obj_add_event_cb(ts->set_btn, set_btn_cb, LV_EVENT_CLICKED, ts);

    lv_obj_t *set_label = lv_label_create(ts->set_btn);
    lv_label_set_text(set_label, "Set");
    lv_obj_set_style_text_color(set_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(set_label, &lv_font_montserrat_20, 0);
    lv_obj_center(set_label);

    // ===== SHARED ELEMENTS =====

    // Time display label - below refresh button (auto) or below rollers (manual)
    ts->time_label = lv_label_create(ts->container);
    lv_label_set_text(ts->time_label, "00:00:00");
    lv_obj_set_style_text_color(ts->time_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(ts->time_label, &lv_font_montserrat_36, 0);
    lv_obj_align(ts->time_label, LV_ALIGN_CENTER, 0, 120);

    // Return button (lower-right corner)
    ts->return_btn = lv_btn_create(ts->container);
    lv_obj_set_size(ts->return_btn, 90, 45);
    lv_obj_align(ts->return_btn, LV_ALIGN_BOTTOM_RIGHT, -25, -25);
    lv_obj_set_style_bg_color(ts->return_btn, lv_color_hex(0x1a3a5c), 0);
    lv_obj_set_style_bg_color(ts->return_btn, lv_color_hex(0x2a5a8c), LV_STATE_PRESSED);
    lv_obj_add_event_cb(ts->return_btn, return_btn_cb, LV_EVENT_CLICKED, ts);

    lv_obj_t *btn_label = lv_label_create(ts->return_btn);
    lv_label_set_text(btn_label, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(btn_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(btn_label, &lv_font_montserrat_14, 0);
    lv_obj_center(btn_label);

    // Start in auto mode - show refresh, hide manual elements
    update_mode_visibility(ts, false);

    return ts;
}

void time_settings_destroy(time_settings_t *ts)
{
    if (!ts) return;

    if (ts->container) {
        lv_obj_del(ts->container);
    }

    s_exit_cb = NULL;
    s_mode_cb = NULL;
    s_refresh_cb = NULL;
    s_set_cb = NULL;
    s_current_ts = NULL;

    free(ts);
}

void time_settings_set_mode(time_settings_t *ts, bool is_manual)
{
    if (!ts || !ts->mode_switch) return;

    if (is_manual) {
        lv_obj_add_state(ts->mode_switch, LV_STATE_CHECKED);
    } else {
        lv_obj_remove_state(ts->mode_switch, LV_STATE_CHECKED);
    }

    update_mode_visibility(ts, is_manual);
}

bool time_settings_get_mode(time_settings_t *ts)
{
    if (!ts || !ts->mode_switch) return false;
    return lv_obj_has_state(ts->mode_switch, LV_STATE_CHECKED);
}

void time_settings_update_time(time_settings_t *ts, int hour, int minute, int second)
{
    if (!ts || !ts->time_label) return;

    char time_str[16];
    snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", hour, minute, second);
    lv_label_set_text(ts->time_label, time_str);
}

void time_settings_set_rollers(time_settings_t *ts, int hour, int minute, int month, int day, int year)
{
    if (!ts) return;

    // Hour: convert 0-23 to roller index (0-23 maps to options 1-24)
    // Hour 0 (midnight) should show as 24, hours 1-23 show as 1-23
    int hour_idx = (hour == 0) ? 23 : (hour - 1);
    if (ts->hour_roller) {
        lv_roller_set_selected(ts->hour_roller, hour_idx, LV_ANIM_OFF);
    }

    // Minute: direct mapping (0-59)
    if (ts->minute_roller) {
        lv_roller_set_selected(ts->minute_roller, minute, LV_ANIM_OFF);
    }

    // Month: 1-12 maps to index 0-11
    if (ts->month_roller) {
        lv_roller_set_selected(ts->month_roller, month - 1, LV_ANIM_OFF);
    }

    // Day: 1-31 maps to index 0-30
    if (ts->day_roller) {
        lv_roller_set_selected(ts->day_roller, day - 1, LV_ANIM_OFF);
    }

    // Year: 2025-2050 maps to index 0-25
    int year_idx = year - 2025;
    if (year_idx < 0) year_idx = 0;
    if (year_idx > 25) year_idx = 25;
    if (ts->year_roller) {
        lv_roller_set_selected(ts->year_roller, year_idx, LV_ANIM_OFF);
    }
}

// Update visibility of elements based on mode
static void update_mode_visibility(time_settings_t *ts, bool is_manual)
{
    if (!ts) return;

    if (is_manual) {
        // Manual mode - hide refresh, show rollers and set button
        if (ts->refresh_btn) lv_obj_add_flag(ts->refresh_btn, LV_OBJ_FLAG_HIDDEN);

        // Show manual elements
        if (ts->hour_label) lv_obj_remove_flag(ts->hour_label, LV_OBJ_FLAG_HIDDEN);
        if (ts->minute_label) lv_obj_remove_flag(ts->minute_label, LV_OBJ_FLAG_HIDDEN);
        if (ts->hour_roller) lv_obj_remove_flag(ts->hour_roller, LV_OBJ_FLAG_HIDDEN);
        if (ts->minute_roller) lv_obj_remove_flag(ts->minute_roller, LV_OBJ_FLAG_HIDDEN);
        if (ts->month_roller) lv_obj_remove_flag(ts->month_roller, LV_OBJ_FLAG_HIDDEN);
        if (ts->day_roller) lv_obj_remove_flag(ts->day_roller, LV_OBJ_FLAG_HIDDEN);
        if (ts->year_roller) lv_obj_remove_flag(ts->year_roller, LV_OBJ_FLAG_HIDDEN);
        if (ts->set_btn) lv_obj_remove_flag(ts->set_btn, LV_OBJ_FLAG_HIDDEN);

        // Move time label down for manual mode
        if (ts->time_label) {
            lv_obj_align(ts->time_label, LV_ALIGN_BOTTOM_MID, 0, -85);
        }

        // Highlight manual label, dim auto label
        if (ts->manual_label) lv_obj_set_style_text_color(ts->manual_label, lv_color_white(), 0);
        if (ts->auto_label) lv_obj_set_style_text_color(ts->auto_label, lv_color_hex(0x666666), 0);
    } else {
        // Auto mode - show refresh, hide rollers and set button
        if (ts->refresh_btn) lv_obj_remove_flag(ts->refresh_btn, LV_OBJ_FLAG_HIDDEN);

        // Hide manual elements
        if (ts->hour_label) lv_obj_add_flag(ts->hour_label, LV_OBJ_FLAG_HIDDEN);
        if (ts->minute_label) lv_obj_add_flag(ts->minute_label, LV_OBJ_FLAG_HIDDEN);
        if (ts->hour_roller) lv_obj_add_flag(ts->hour_roller, LV_OBJ_FLAG_HIDDEN);
        if (ts->minute_roller) lv_obj_add_flag(ts->minute_roller, LV_OBJ_FLAG_HIDDEN);
        if (ts->month_roller) lv_obj_add_flag(ts->month_roller, LV_OBJ_FLAG_HIDDEN);
        if (ts->day_roller) lv_obj_add_flag(ts->day_roller, LV_OBJ_FLAG_HIDDEN);
        if (ts->year_roller) lv_obj_add_flag(ts->year_roller, LV_OBJ_FLAG_HIDDEN);
        if (ts->set_btn) lv_obj_add_flag(ts->set_btn, LV_OBJ_FLAG_HIDDEN);

        // Move time label to center position for auto mode
        if (ts->time_label) {
            lv_obj_align(ts->time_label, LV_ALIGN_CENTER, 0, 120);
        }

        // Highlight auto label, dim manual label
        if (ts->auto_label) lv_obj_set_style_text_color(ts->auto_label, lv_color_white(), 0);
        if (ts->manual_label) lv_obj_set_style_text_color(ts->manual_label, lv_color_hex(0x666666), 0);
    }

    // Time label always visible
    if (ts->time_label) lv_obj_remove_flag(ts->time_label, LV_OBJ_FLAG_HIDDEN);
}

// Return button callback
static void return_btn_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        if (s_exit_cb) {
            s_exit_cb();
        }
    }
}

// Switch value changed callback
static void switch_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
        time_settings_t *ts = (time_settings_t *)lv_event_get_user_data(e);
        bool is_manual = lv_obj_has_state(ts->mode_switch, LV_STATE_CHECKED);

        // Update UI visibility
        update_mode_visibility(ts, is_manual);

        // Notify via callback (main.cpp will set roller values if switching to manual)
        if (s_mode_cb) {
            s_mode_cb(is_manual);
        }
    }
}

// Refresh button callback
static void refresh_btn_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        if (s_refresh_cb) {
            s_refresh_cb();
        }
    }
}

// Set button callback
static void set_btn_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        time_settings_t *ts = (time_settings_t *)lv_event_get_user_data(e);
        if (!ts || !s_set_cb) return;

        // Get values from rollers
        // Hour: index 0-23 maps to 1-24, convert back to 0-23
        int hour_idx = lv_roller_get_selected(ts->hour_roller);
        int hour = (hour_idx == 23) ? 0 : (hour_idx + 1);  // 24 -> 0, 1-23 -> 1-23

        int minute = lv_roller_get_selected(ts->minute_roller);  // 0-59 direct
        int month = lv_roller_get_selected(ts->month_roller) + 1;  // index + 1 = 1-12
        int day = lv_roller_get_selected(ts->day_roller) + 1;  // index + 1 = 1-31
        int year = lv_roller_get_selected(ts->year_roller) + 2025;  // index + 2025

        // Call the set callback with the values
        s_set_cb(hour, minute, month, day, year);
    }
}
