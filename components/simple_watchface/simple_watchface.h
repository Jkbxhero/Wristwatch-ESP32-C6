/**
 * @file simple_watchface.h
 * @brief Simple text-based watchface for testing
 *
 * Displays time and date as text in the center of the screen.
 * Minimal implementation for build testing before adding complex UI.
 */

#ifndef SIMPLE_WATCHFACE_H
#define SIMPLE_WATCHFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

// Simple watchface structure
typedef struct {
    lv_obj_t *screen;
    lv_obj_t *time_label;      // "12:34:56"
    lv_obj_t *date_label;      // "THU 24"
    lv_obj_t *ampm_label;      // "AM" or "PM" (optional)
} simple_watchface_t;

/**
 * @brief Create the simple watchface
 * @param parent Parent object (usually lv_screen_active())
 * @return Pointer to watchface structure
 */
simple_watchface_t* simple_watchface_create(lv_obj_t *parent);

/**
 * @brief Update time display
 * @param wf Watchface pointer
 * @param hour Hour (0-23)
 * @param minute Minute (0-59)
 * @param second Second (0-59)
 */
void simple_watchface_set_time(simple_watchface_t *wf, int hour, int minute, int second);

/**
 * @brief Update date display
 * @param wf Watchface pointer
 * @param day_name 3-letter day name (e.g., "THU")
 * @param day_num Day of month (1-31)
 */
void simple_watchface_set_date(simple_watchface_t *wf, const char *day_name, int day_num);

/**
 * @brief Destroy watchface and free resources
 * @param wf Watchface pointer
 */
void simple_watchface_destroy(simple_watchface_t *wf);

/**
 * @brief Update accelerometer display (stub - simple watchface doesn't show accel)
 * @param wf Watchface pointer
 * @param x X-axis acceleration
 * @param y Y-axis acceleration
 * @param z Z-axis acceleration
 */
void simple_watchface_set_accel(simple_watchface_t *wf, float x, float y, float z);

// Note: The current_watchface_* aliases have been removed.
// main.cpp now explicitly calls the correct watchface functions
// based on current_watchface_type for dynamic watchface selection.

#ifdef __cplusplus
}
#endif

#endif // SIMPLE_WATCHFACE_H
