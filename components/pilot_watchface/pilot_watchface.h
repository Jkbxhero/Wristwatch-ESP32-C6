/**
 * @file pilot_watchface.h
 * @brief Pilot/Aviation style analog watchface for ESP32 + LVGL 9.x
 * 
 * Display: 410x502 rectangular with rounded corners
 * Style: Aviation chronograph with 3 sub-dials for accelerometer
 * 
 * Features:
 * - Dark blue gradient background
 * - Aviation-style hour markers and numbers
 * - 3 sub-dials showing X, Y, Z accelerometer data
 * - Day (3-letter) and Date (2-digit) display
 * - Rotating clock hands (hour, minute, second)
 * - Charging indicator
 */

#ifndef PILOT_WATCHFACE_H
#define PILOT_WATCHFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

// Display dimensions
#define DISPLAY_WIDTH   410
#define DISPLAY_HEIGHT  502

// Main dial dimensions - RECTANGULAR to fill display
#define DIAL_WIDTH      405
#define DIAL_HEIGHT     492
#define DIAL_CENTER_X   (DISPLAY_WIDTH / 2)
#define DIAL_CENTER_Y   (DISPLAY_HEIGHT / 2)
#define DIAL_CORNER_RADIUS  110  // Rounded corner radius - adjust to match display

// Half dimensions for tick calculations
#define DIAL_HALF_W     (DIAL_WIDTH / 2)
#define DIAL_HALF_H     (DIAL_HEIGHT / 2)

// Sub-dial dimensions - repositioned for rectangular layout
#define SUBDIAL_DIAMETER    100
#define SUBDIAL_12_X        (DIAL_CENTER_X)           // Top sub-dial (Z accel)
#define SUBDIAL_12_Y        (DIAL_CENTER_Y - 100)
#define SUBDIAL_9_X         (DIAL_CENTER_X - 85)     // Left sub-dial (X accel)
#define SUBDIAL_9_Y         (DIAL_CENTER_Y + 0)
#define SUBDIAL_6_X         (DIAL_CENTER_X)           // Bottom sub-dial (Y accel)  
#define SUBDIAL_6_Y         (DIAL_CENTER_Y + 100)

// Date window position (right side, 3 o'clock area - no longer competing with Z)
#define DATE_WINDOW_X       (DIAL_CENTER_X + 85)
#define DATE_WINDOW_Y       (DIAL_CENTER_Y + 0)

// Clock hand lengths (from center) - slightly longer for rectangular
#define HOUR_HAND_LENGTH    85
#define MINUTE_HAND_LENGTH  130
#define SECOND_HAND_LENGTH  160

// Clock hand widths
#define HOUR_HAND_WIDTH     8
#define MINUTE_HAND_WIDTH   6
#define SECOND_HAND_WIDTH   2

// Colors (aviation/pilot theme with dark blue)
#define COLOR_BACKGROUND_DARK   lv_color_hex(0x0a1628)  // Dark navy blue
#define COLOR_BACKGROUND_LIGHT  lv_color_hex(0x1a3a5c)  // Lighter blue for gradient
#define COLOR_DIAL_RING         lv_color_hex(0x2a4a6c)  // Dial outer ring
#define COLOR_TICK_MAJOR        lv_color_hex(0xE8E8E8)  // White-ish for major ticks
#define COLOR_TICK_MINOR        lv_color_hex(0x888888)  // Gray for minor ticks
#define COLOR_NUMBERS           lv_color_hex(0xFFFFFF)  // White numbers
#define COLOR_HOUR_HAND         lv_color_hex(0xE8E8E8)  // Light gray
#define COLOR_MINUTE_HAND       lv_color_hex(0xFFFFFF)  // White
#define COLOR_SECOND_HAND       lv_color_hex(0xFF4444)  // Red (classic aviation)
#define COLOR_SUBDIAL_BG        lv_color_hex(0x0d1f33)  // Darker than main
#define COLOR_SUBDIAL_RING      lv_color_hex(0x3a5a7c)  // Subtle ring
#define COLOR_DATE_BG           lv_color_hex(0x1a1a1a)  // Dark date window
#define COLOR_DATE_TEXT         lv_color_hex(0xFFFFFF)  // White date text
#define COLOR_CHARGING          lv_color_hex(0x44FF44)  // Green charging indicator

// Watchface structure - holds all UI elements
typedef struct {
    // Main container
    lv_obj_t *screen;
    
    // Background and dial
    lv_obj_t *dial_bg;
    
    // Hour markers (12 total)
    lv_obj_t *hour_markers[12];
    lv_obj_t *hour_labels[12];     // 12, 1, 2, 3... numbers
    
    // Minute tick marks (60 total)
    lv_obj_t *minute_ticks[60];
    
    // Sub-dials for accelerometer
    lv_obj_t *subdial_x;           // 9 o'clock position - X axis
    lv_obj_t *subdial_y;           // 6 o'clock position - Y axis
    lv_obj_t *subdial_z;           // 3 o'clock position - Z axis
    lv_obj_t *subdial_x_label;     // "X" label
    lv_obj_t *subdial_y_label;     // "Y" label
    lv_obj_t *subdial_z_label;     // "Z" label
    lv_obj_t *subdial_x_value;     // X value display
    lv_obj_t *subdial_y_value;     // Y value display
    lv_obj_t *subdial_z_value;     // Z value display
    lv_obj_t *subdial_x_needle;    // X needle
    lv_obj_t *subdial_y_needle;    // Y needle
    lv_obj_t *subdial_z_needle;    // Z needle
    
    // Date display
    lv_obj_t *date_window;
    lv_obj_t *day_label;           // "THU"
    lv_obj_t *date_label;          // "24"
    
    // Clock hands
    lv_obj_t *hour_hand;
    lv_obj_t *minute_hand;
    lv_obj_t *second_hand;
    lv_obj_t *center_cap;          // Center circle covering hand pivots
    
    // Status indicators
    lv_obj_t *charging_label;
    
} pilot_watchface_t;

/**
 * @brief Initialize and create the pilot watchface
 * @param parent Parent object (usually lv_screen_active())
 * @return Pointer to the watchface structure
 */
pilot_watchface_t* pilot_watchface_create(lv_obj_t *parent);

/**
 * @brief Update the time display
 * @param wf Watchface pointer
 * @param hour Hour (0-23)
 * @param minute Minute (0-59)
 * @param second Second (0-59)
 */
void pilot_watchface_set_time(pilot_watchface_t *wf, int hour, int minute, int second);

/**
 * @brief Update the date display
 * @param wf Watchface pointer
 * @param day_name 3-letter day name (e.g., "THU")
 * @param day_num Day of month (1-31)
 */
void pilot_watchface_set_date(pilot_watchface_t *wf, const char *day_name, int day_num);

/**
 * @brief Update accelerometer sub-dials
 * @param wf Watchface pointer
 * @param x X-axis acceleration (m/s²)
 * @param y Y-axis acceleration (m/s²)
 * @param z Z-axis acceleration (m/s²)
 */
void pilot_watchface_set_accel(pilot_watchface_t *wf, float x, float y, float z);

/**
 * @brief Update charging status
 * @param wf Watchface pointer
 * @param is_charging True if charging
 * @param battery_percent Battery percentage (0-100)
 */
void pilot_watchface_set_charging(pilot_watchface_t *wf, bool is_charging, int battery_percent);

/**
 * @brief Clean up watchface resources
 * @param wf Watchface pointer
 */
void pilot_watchface_destroy(pilot_watchface_t *wf);

#ifdef __cplusplus
}
#endif

#endif // PILOT_WATCHFACE_H
