/**
 * @file time_settings.h
 * @brief Time settings app for C6Watch
 *
 * Allows user to switch between auto (NTP) and manual time setting modes.
 * In auto mode, provides a refresh button to re-sync with NTP.
 * In manual mode, provides rollers to set time and date.
 */

#ifndef TIME_SETTINGS_H
#define TIME_SETTINGS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

// Time settings screen structure
typedef struct {
    lv_obj_t *container;        // Main container
    lv_obj_t *title_label;      // "Time Settings" title
    lv_obj_t *auto_label;       // "Auto" label
    lv_obj_t *manual_label;     // "Manual" label
    lv_obj_t *mode_switch;      // Switch widget (off=auto, on=manual)

    // Auto mode elements
    lv_obj_t *refresh_btn;      // Refresh button (auto mode only)

    // Manual mode elements
    lv_obj_t *hour_label;       // "Hour" label
    lv_obj_t *minute_label;     // "Min" label
    lv_obj_t *hour_roller;      // Hours roller (1-24)
    lv_obj_t *minute_roller;    // Minutes roller (0-59)
    lv_obj_t *month_roller;     // Month roller (1-12)
    lv_obj_t *day_roller;       // Day roller (1-31)
    lv_obj_t *year_roller;      // Year roller (2025-2050)
    lv_obj_t *set_btn;          // Set button (manual mode only)

    // Shared elements
    lv_obj_t *time_label;       // Current time display
    lv_obj_t *return_btn;       // Return button
} time_settings_t;

// Callback type for when user exits time settings
typedef void (*time_settings_exit_cb_t)(void);

// Callback type for when mode changes
typedef void (*time_settings_mode_cb_t)(bool is_manual);

// Callback type for refresh button press
typedef void (*time_settings_refresh_cb_t)(void);

// Callback type for set button press (manual mode)
// Parameters: hour (1-24), minute (0-59), month (1-12), day (1-31), year (2025-2050)
typedef void (*time_settings_set_cb_t)(int hour, int minute, int month, int day, int year);

/**
 * @brief Create the time settings screen
 * @param parent Parent object (usually lv_scr_act())
 * @param exit_cb Callback when user presses return (can be NULL)
 * @param mode_cb Callback when mode switch changes (can be NULL)
 * @param refresh_cb Callback when refresh button pressed (can be NULL)
 * @param set_cb Callback when set button pressed in manual mode (can be NULL)
 * @return Pointer to time_settings structure, or NULL on failure
 */
time_settings_t* time_settings_create(lv_obj_t *parent,
                                       time_settings_exit_cb_t exit_cb,
                                       time_settings_mode_cb_t mode_cb,
                                       time_settings_refresh_cb_t refresh_cb,
                                       time_settings_set_cb_t set_cb);

/**
 * @brief Destroy time settings screen and free resources
 * @param ts Time settings pointer
 */
void time_settings_destroy(time_settings_t *ts);

/**
 * @brief Set the current mode state
 * @param ts Time settings pointer
 * @param is_manual true for manual mode, false for auto
 */
void time_settings_set_mode(time_settings_t *ts, bool is_manual);

/**
 * @brief Get the current mode state
 * @param ts Time settings pointer
 * @return true if manual mode, false if auto
 */
bool time_settings_get_mode(time_settings_t *ts);

/**
 * @brief Update the time display
 * @param ts Time settings pointer
 * @param hour Hour (0-23)
 * @param minute Minute (0-59)
 * @param second Second (0-59)
 */
void time_settings_update_time(time_settings_t *ts, int hour, int minute, int second);

/**
 * @brief Set the roller values to current date/time (call when switching to manual)
 * @param ts Time settings pointer
 * @param hour Hour (0-23, will be converted to 1-24 for display)
 * @param minute Minute (0-59)
 * @param month Month (1-12)
 * @param day Day (1-31)
 * @param year Year (2025-2050)
 */
void time_settings_set_rollers(time_settings_t *ts, int hour, int minute, int month, int day, int year);

#ifdef __cplusplus
}
#endif

#endif // TIME_SETTINGS_H
