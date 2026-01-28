/**
 * @file pilot_watchface.c
 * @brief Pilot/Aviation style analog watchface implementation
 */

#include "pilot_watchface.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_heap_caps.h"

static const char *TAG = "pilot_watchface";

// Minimum heap required to create pilot watchface (estimated ~50KB for all objects)
#define MIN_HEAP_FOR_WATCHFACE  40000

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Convert angle in degrees to radians
 */
static float deg_to_rad(float deg) {
    return deg * (float)M_PI / 180.0f;
}

/**
 * @brief Calculate point on circle given center, radius, and angle
 * @param cx Center X
 * @param cy Center Y
 * @param radius Radius
 * @param angle_deg Angle in degrees (0 = 12 o'clock, clockwise)
 * @param out_x Output X coordinate
 * @param out_y Output Y coordinate
 */
static void point_on_circle(int cx, int cy, int radius, float angle_deg, 
                            int *out_x, int *out_y) {
    // Convert from clock angle (0=12 o'clock) to math angle (0=3 o'clock)
    float math_angle = deg_to_rad(angle_deg - 90.0f);
    *out_x = cx + (int)(radius * cosf(math_angle));
    *out_y = cy + (int)(radius * sinf(math_angle));
}

/**
 * @brief Calculate point on ellipse given center, radii, and angle
 * @param cx Center X
 * @param cy Center Y
 * @param radius_x Horizontal radius
 * @param radius_y Vertical radius
 * @param angle_deg Angle in degrees (0 = 12 o'clock, clockwise)
 * @param out_x Output X coordinate
 * @param out_y Output Y coordinate
 */
static void point_on_ellipse(int cx, int cy, int radius_x, int radius_y, float angle_deg, 
                             int *out_x, int *out_y) {
    // Convert from clock angle (0=12 o'clock) to math angle (0=3 o'clock)
    float math_angle = deg_to_rad(angle_deg - 90.0f);
    *out_x = cx + (int)(radius_x * cosf(math_angle));
    *out_y = cy + (int)(radius_y * sinf(math_angle));
}

/**
 * @brief Calculate point on rounded rectangle perimeter
 * @param cx Center X
 * @param cy Center Y
 * @param half_width Half of rectangle width
 * @param half_height Half of rectangle height
 * @param corner_radius Corner radius
 * @param angle_deg Angle in degrees (0 = 12 o'clock, clockwise)
 * @param out_x Output X coordinate
 * @param out_y Output Y coordinate
 */
static void point_on_rounded_rect(int cx, int cy, int half_width, int half_height, 
                                   int corner_radius, float angle_deg,
                                   int *out_x, int *out_y) {
    // Normalize angle to 0-360
    while (angle_deg < 0) angle_deg += 360.0f;
    while (angle_deg >= 360) angle_deg -= 360.0f;
    
    // Convert to math angle (0 = right, counter-clockwise)
    float math_angle = deg_to_rad(90.0f - angle_deg);
    
    // Direction vector
    float dx = cosf(math_angle);
    float dy = -sinf(math_angle);  // Negative because Y increases downward
    
    // Inner rectangle bounds (where straight edges are)
    int inner_w = half_width - corner_radius;
    int inner_h = half_height - corner_radius;
    
    // Find intersection with rounded rectangle
    // Check which edge/corner the ray intersects
    float t_right = (dx > 0.001f) ? (half_width / dx) : 99999.0f;
    float t_left = (dx < -0.001f) ? (-half_width / dx) : 99999.0f;
    float t_top = (dy < -0.001f) ? (-half_height / dy) : 99999.0f;
    float t_bottom = (dy > 0.001f) ? (half_height / dy) : 99999.0f;
    
    // Find minimum positive t
    float t = 99999.0f;
    if (t_right > 0 && t_right < t) t = t_right;
    if (t_left > 0 && t_left < t) t = t_left;
    if (t_top > 0 && t_top < t) t = t_top;
    if (t_bottom > 0 && t_bottom < t) t = t_bottom;
    
    // Calculate intersection point
    float px = t * dx;
    float py = t * dy;
    
    // Check if we're in a corner region and need to adjust for rounding
    int in_corner = 0;
    float corner_cx = 0, corner_cy = 0;
    
    if (px > inner_w && py < -inner_h) {
        // Top-right corner
        in_corner = 1;
        corner_cx = inner_w;
        corner_cy = -inner_h;
    } else if (px > inner_w && py > inner_h) {
        // Bottom-right corner
        in_corner = 1;
        corner_cx = inner_w;
        corner_cy = inner_h;
    } else if (px < -inner_w && py > inner_h) {
        // Bottom-left corner
        in_corner = 1;
        corner_cx = -inner_w;
        corner_cy = inner_h;
    } else if (px < -inner_w && py < -inner_h) {
        // Top-left corner
        in_corner = 1;
        corner_cx = -inner_w;
        corner_cy = -inner_h;
    }
    
    if (in_corner) {
        // Project onto corner circle
        float to_corner_x = px - corner_cx;
        float to_corner_y = py - corner_cy;
        float dist = sqrtf(to_corner_x * to_corner_x + to_corner_y * to_corner_y);
        if (dist > 0.001f) {
            px = corner_cx + (to_corner_x / dist) * corner_radius;
            py = corner_cy + (to_corner_y / dist) * corner_radius;
        }
    }
    
    *out_x = cx + (int)px;
    *out_y = cy + (int)py;
}

/**
 * @brief Create a line object for tick marks or hands
 */
static lv_obj_t* create_line(lv_obj_t *parent, int x1, int y1, int x2, int y2,
                              int width, lv_color_t color) {
    lv_obj_t *line = lv_line_create(parent);
    
    // Allocate points (must persist - use static or malloc)
    static lv_point_precise_t points[60][2];  // Max 60 tick marks
    static int point_idx = 0;
    
    // Use modulo to recycle points array
    int idx = point_idx % 60;
    points[idx][0].x = x1;
    points[idx][0].y = y1;
    points[idx][1].x = x2;
    points[idx][1].y = y2;
    point_idx++;
    
    lv_line_set_points(line, points[idx], 2);
    lv_obj_set_style_line_width(line, width, 0);
    lv_obj_set_style_line_color(line, color, 0);
    lv_obj_set_style_line_rounded(line, true, 0);
    
    return line;
}

// ============================================================================
// Sub-dial Creation
// ============================================================================

/**
 * @brief Create a sub-dial for accelerometer display
 */
static lv_obj_t* create_subdial(lv_obj_t *parent, int cx, int cy, int diameter,
                                 lv_obj_t **out_label, lv_obj_t **out_value,
                                 lv_obj_t **out_needle, const char *axis_name) {
    // Create circular background
    lv_obj_t *subdial = lv_obj_create(parent);
    lv_obj_set_size(subdial, diameter, diameter);
    lv_obj_set_pos(subdial, cx - diameter/2, cy - diameter/2);
    lv_obj_set_style_radius(subdial, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(subdial, COLOR_SUBDIAL_BG, 0);
    lv_obj_set_style_bg_opa(subdial, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(subdial, COLOR_SUBDIAL_RING, 0);
    lv_obj_set_style_border_width(subdial, 2, 0);
    lv_obj_set_scrollbar_mode(subdial, LV_SCROLLBAR_MODE_OFF);
    lv_obj_remove_flag(subdial, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(subdial, LV_OBJ_FLAG_EVENT_BUBBLE);  // Allow events to bubble up
    
    // Add small tick marks around edge (8 ticks)
    for (int i = 0; i < 8; i++) {
        float angle = i * 45.0f;
        int inner_r = diameter/2 - 8;
        int outer_r = diameter/2 - 3;
        int x1, y1, x2, y2;
        point_on_circle(diameter/2, diameter/2, inner_r, angle, &x1, &y1);
        point_on_circle(diameter/2, diameter/2, outer_r, angle, &x2, &y2);
        
        lv_obj_t *tick = lv_line_create(subdial);
        static lv_point_precise_t subdial_ticks[24][2];  // 3 subdials * 8 ticks
        static int tick_idx = 0;
        int tidx = tick_idx % 24;
        subdial_ticks[tidx][0].x = x1;
        subdial_ticks[tidx][0].y = y1;
        subdial_ticks[tidx][1].x = x2;
        subdial_ticks[tidx][1].y = y2;
        tick_idx++;
        
        lv_line_set_points(tick, subdial_ticks[tidx], 2);
        lv_obj_set_style_line_width(tick, 1, 0);
        lv_obj_set_style_line_color(tick, COLOR_TICK_MINOR, 0);
    }
    
    // Axis label at top of subdial
    *out_label = lv_label_create(subdial);
    lv_label_set_text(*out_label, axis_name);
    lv_obj_set_style_text_color(*out_label, COLOR_TICK_MINOR, 0);
    lv_obj_set_style_text_font(*out_label, &lv_font_montserrat_12, 0);
    lv_obj_align(*out_label, LV_ALIGN_TOP_MID, 0, 8);
    
    // Value at bottom of subdial
    *out_value = lv_label_create(subdial);
    lv_label_set_text(*out_value, "0.0");
    lv_obj_set_style_text_color(*out_value, COLOR_NUMBERS, 0);
    lv_obj_set_style_text_font(*out_value, &lv_font_montserrat_10, 0);
    lv_obj_align(*out_value, LV_ALIGN_BOTTOM_MID, 0, -6);
    
    // Create needle (starts pointing up)
    *out_needle = lv_line_create(subdial);
    static lv_point_precise_t needle_pts[3][2];
    static int needle_idx = 0;
    int nidx = needle_idx % 3;
    needle_pts[nidx][0].x = diameter/2;
    needle_pts[nidx][0].y = diameter/2;
    needle_pts[nidx][1].x = diameter/2;
    needle_pts[nidx][1].y = 10;
    needle_idx++;
    
    lv_line_set_points(*out_needle, needle_pts[nidx], 2);
    lv_obj_set_style_line_width(*out_needle, 2, 0);
    lv_obj_set_style_line_color(*out_needle, COLOR_SECOND_HAND, 0);
    lv_obj_set_style_line_rounded(*out_needle, true, 0);
    
    return subdial;
}

// ============================================================================
// Main Watchface Creation
// ============================================================================

pilot_watchface_t* pilot_watchface_create(lv_obj_t *parent) {
    // Check available heap before attempting to create complex watchface
    size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "Creating pilot watchface, free heap: %u bytes", free_heap);

    if (free_heap < MIN_HEAP_FOR_WATCHFACE) {
        ESP_LOGE(TAG, "Insufficient heap for pilot watchface! Need %d, have %u",
                 MIN_HEAP_FOR_WATCHFACE, free_heap);
        return NULL;
    }

    pilot_watchface_t *wf = (pilot_watchface_t*)malloc(sizeof(pilot_watchface_t));
    if (!wf) {
        ESP_LOGE(TAG, "Failed to allocate watchface structure");
        return NULL;
    }
    memset(wf, 0, sizeof(pilot_watchface_t));

    wf->screen = parent;

    // ========================================================================
    // Background with gradient
    // ========================================================================
    lv_obj_set_style_bg_color(parent, COLOR_BACKGROUND_DARK, 0);
    lv_obj_set_style_bg_grad_color(parent, COLOR_BACKGROUND_LIGHT, 0);
    lv_obj_set_style_bg_grad_dir(parent, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);

    // ========================================================================
    // Main dial - RECTANGULAR with rounded corners
    // ========================================================================
    wf->dial_bg = lv_obj_create(parent);
    if (!wf->dial_bg) {
        ESP_LOGE(TAG, "Failed to create dial background - out of memory");
        free(wf);
        return NULL;
    }
    lv_obj_set_size(wf->dial_bg, DIAL_WIDTH, DIAL_HEIGHT);
    lv_obj_center(wf->dial_bg);
    lv_obj_set_style_radius(wf->dial_bg, DIAL_CORNER_RADIUS, 0);  // Rounded corners
    lv_obj_set_style_bg_color(wf->dial_bg, COLOR_BACKGROUND_DARK, 0);
    lv_obj_set_style_bg_opa(wf->dial_bg, LV_OPA_80, 0);
    lv_obj_set_style_border_color(wf->dial_bg, COLOR_DIAL_RING, 0);
    lv_obj_set_style_border_width(wf->dial_bg, 4, 0);
    lv_obj_set_scrollbar_mode(wf->dial_bg, LV_SCROLLBAR_MODE_OFF);
    lv_obj_remove_flag(wf->dial_bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(wf->dial_bg, LV_OBJ_FLAG_EVENT_BUBBLE);  // Allow touch events to bubble to parent
    
    // ========================================================================
    // Hour markers and numbers - follow ROUNDED RECTANGLE
    // ========================================================================
    const char *hour_nums[] = {"12", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11"};
    
    for (int i = 0; i < 12; i++) {
        float angle = i * 30.0f;  // 360/12 = 30 degrees per hour
        
        // Major tick mark - follows rounded rectangle perimeter
        int inner_x, inner_y, outer_x, outer_y;
        point_on_rounded_rect(DIAL_CENTER_X, DIAL_CENTER_Y, 
                              DIAL_HALF_W - 25, DIAL_HALF_H - 25, 
                              DIAL_CORNER_RADIUS - 20, angle, &inner_x, &inner_y);
        point_on_rounded_rect(DIAL_CENTER_X, DIAL_CENTER_Y, 
                              DIAL_HALF_W - 3, DIAL_HALF_H - 3, 
                              DIAL_CORNER_RADIUS - 5, angle, &outer_x, &outer_y);
        
        wf->hour_markers[i] = lv_line_create(parent);
        static lv_point_precise_t hour_tick_pts[12][2];
        hour_tick_pts[i][0].x = inner_x;
        hour_tick_pts[i][0].y = inner_y;
        hour_tick_pts[i][1].x = outer_x;
        hour_tick_pts[i][1].y = outer_y;
        lv_line_set_points(wf->hour_markers[i], hour_tick_pts[i], 2);
        lv_obj_set_style_line_width(wf->hour_markers[i], 6, 0);
        lv_obj_set_style_line_color(wf->hour_markers[i], COLOR_TICK_MAJOR, 0);
        lv_obj_set_style_line_rounded(wf->hour_markers[i], true, 0);
        
        // Hour number label - positioned inside tick marks
        int label_x, label_y;
        point_on_rounded_rect(DIAL_CENTER_X, DIAL_CENTER_Y, 
                              DIAL_HALF_W - 50, DIAL_HALF_H - 50, 
                              DIAL_CORNER_RADIUS - 40, angle, &label_x, &label_y);
        
        wf->hour_labels[i] = lv_label_create(parent);
        lv_label_set_text(wf->hour_labels[i], hour_nums[i]);
        lv_obj_set_style_text_color(wf->hour_labels[i], COLOR_NUMBERS, 0);
        lv_obj_set_style_text_font(wf->hour_labels[i], &lv_font_montserrat_30, 0);
        lv_obj_set_pos(wf->hour_labels[i], label_x - 12, label_y - 12);  // Center the label
    }
    
    // ========================================================================
    // Minute tick marks (60 total, skip hour positions) - ROUNDED RECTANGLE
    // ========================================================================
    for (int i = 0; i < 60; i++) {
        if (i % 5 == 0) continue;  // Skip hour marker positions
        
        float angle = i * 6.0f;  // 360/60 = 6 degrees per minute
        
        int inner_x, inner_y, outer_x, outer_y;
        point_on_rounded_rect(DIAL_CENTER_X, DIAL_CENTER_Y, 
                              DIAL_HALF_W - 12, DIAL_HALF_H - 12, 
                              DIAL_CORNER_RADIUS - 8, angle, &inner_x, &inner_y);
        point_on_rounded_rect(DIAL_CENTER_X, DIAL_CENTER_Y, 
                              DIAL_HALF_W - 2, DIAL_HALF_H - 2, 
                              DIAL_CORNER_RADIUS - 3, angle, &outer_x, &outer_y);
        
        wf->minute_ticks[i] = lv_line_create(parent);
        static lv_point_precise_t min_tick_pts[60][2];
        min_tick_pts[i][0].x = inner_x;
        min_tick_pts[i][0].y = inner_y;
        min_tick_pts[i][1].x = outer_x;
        min_tick_pts[i][1].y = outer_y;
        lv_line_set_points(wf->minute_ticks[i], min_tick_pts[i], 2);
        lv_obj_set_style_line_width(wf->minute_ticks[i], 2, 0);
        lv_obj_set_style_line_color(wf->minute_ticks[i], COLOR_TICK_MINOR, 0);
    }
    
    // ========================================================================
    // Sub-dials for accelerometer (X at 9, Y at 6, Z at 12)
    // ========================================================================
    wf->subdial_x = create_subdial(parent, SUBDIAL_9_X, SUBDIAL_9_Y, SUBDIAL_DIAMETER,
                                    &wf->subdial_x_label, &wf->subdial_x_value,
                                    &wf->subdial_x_needle, "X");
    
    wf->subdial_y = create_subdial(parent, SUBDIAL_6_X, SUBDIAL_6_Y, SUBDIAL_DIAMETER,
                                    &wf->subdial_y_label, &wf->subdial_y_value,
                                    &wf->subdial_y_needle, "Y");
    
    wf->subdial_z = create_subdial(parent, SUBDIAL_12_X, SUBDIAL_12_Y, SUBDIAL_DIAMETER,
                                    &wf->subdial_z_label, &wf->subdial_z_value,
                                    &wf->subdial_z_needle, "Z");
    
    // ========================================================================
    // Date window (day name + date number)
    // ========================================================================
    wf->date_window = lv_obj_create(parent);
    lv_obj_set_size(wf->date_window, 60, 40);
    lv_obj_set_pos(wf->date_window, DATE_WINDOW_X - 30, DATE_WINDOW_Y - 20);
    lv_obj_set_style_radius(wf->date_window, 4, 0);
    lv_obj_set_style_bg_color(wf->date_window, COLOR_DATE_BG, 0);
    lv_obj_set_style_bg_opa(wf->date_window, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(wf->date_window, COLOR_SUBDIAL_RING, 0);
    lv_obj_set_style_border_width(wf->date_window, 1, 0);
    lv_obj_set_scrollbar_mode(wf->date_window, LV_SCROLLBAR_MODE_OFF);
    lv_obj_remove_flag(wf->date_window, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(wf->date_window, LV_OBJ_FLAG_EVENT_BUBBLE);  // Allow events to bubble up
    
    // Day name (e.g., "THU")
    wf->day_label = lv_label_create(wf->date_window);
    lv_label_set_text(wf->day_label, "THU");
    lv_obj_set_style_text_color(wf->day_label, COLOR_SECOND_HAND, 0);  // Red like reference
    lv_obj_set_style_text_font(wf->day_label, &lv_font_montserrat_12, 0);
    lv_obj_align(wf->day_label, LV_ALIGN_TOP_MID, 0, 2);
    
    // Date number (e.g., "24")
    wf->date_label = lv_label_create(wf->date_window);
    lv_label_set_text(wf->date_label, "24");
    lv_obj_set_style_text_color(wf->date_label, COLOR_DATE_TEXT, 0);
    lv_obj_set_style_text_font(wf->date_label, &lv_font_montserrat_18, 0);
    lv_obj_align(wf->date_label, LV_ALIGN_BOTTOM_MID, 0, -2);
    
    // ========================================================================
    // Clock hands (hour, minute, second) - drawn as lines
    // ========================================================================
    
    // Hour hand (shortest, widest)
    wf->hour_hand = lv_line_create(parent);
    static lv_point_precise_t hour_pts[2];
    hour_pts[0].x = DIAL_CENTER_X;
    hour_pts[0].y = DIAL_CENTER_Y;
    hour_pts[1].x = DIAL_CENTER_X;
    hour_pts[1].y = DIAL_CENTER_Y - HOUR_HAND_LENGTH;
    lv_line_set_points(wf->hour_hand, hour_pts, 2);
    lv_obj_set_style_line_width(wf->hour_hand, HOUR_HAND_WIDTH, 0);
    lv_obj_set_style_line_color(wf->hour_hand, COLOR_HOUR_HAND, 0);
    lv_obj_set_style_line_rounded(wf->hour_hand, true, 0);
    
    // Minute hand (medium length)
    wf->minute_hand = lv_line_create(parent);
    static lv_point_precise_t minute_pts[2];
    minute_pts[0].x = DIAL_CENTER_X;
    minute_pts[0].y = DIAL_CENTER_Y;
    minute_pts[1].x = DIAL_CENTER_X;
    minute_pts[1].y = DIAL_CENTER_Y - MINUTE_HAND_LENGTH;
    lv_line_set_points(wf->minute_hand, minute_pts, 2);
    lv_obj_set_style_line_width(wf->minute_hand, MINUTE_HAND_WIDTH, 0);
    lv_obj_set_style_line_color(wf->minute_hand, COLOR_MINUTE_HAND, 0);
    lv_obj_set_style_line_rounded(wf->minute_hand, true, 0);
    
    // Second hand (longest, thinnest, red)
    wf->second_hand = lv_line_create(parent);
    static lv_point_precise_t second_pts[2];
    second_pts[0].x = DIAL_CENTER_X;
    second_pts[0].y = DIAL_CENTER_Y + 20;  // Small tail
    second_pts[1].x = DIAL_CENTER_X;
    second_pts[1].y = DIAL_CENTER_Y - SECOND_HAND_LENGTH;
    lv_line_set_points(wf->second_hand, second_pts, 2);
    lv_obj_set_style_line_width(wf->second_hand, SECOND_HAND_WIDTH, 0);
    lv_obj_set_style_line_color(wf->second_hand, COLOR_SECOND_HAND, 0);
    lv_obj_set_style_line_rounded(wf->second_hand, true, 0);
    
    // Center cap (covers the pivot point)
    wf->center_cap = lv_obj_create(parent);
    lv_obj_set_size(wf->center_cap, 16, 16);
    lv_obj_set_pos(wf->center_cap, DIAL_CENTER_X - 8, DIAL_CENTER_Y - 8);
    lv_obj_set_style_radius(wf->center_cap, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(wf->center_cap, COLOR_SECOND_HAND, 0);
    lv_obj_set_style_bg_opa(wf->center_cap, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(wf->center_cap, 0, 0);
    
    // ========================================================================
    // Charging indicator (top of screen)
    // ========================================================================
    wf->charging_label = lv_label_create(parent);
    lv_label_set_text(wf->charging_label, "");  // Hidden by default
    lv_obj_set_style_text_color(wf->charging_label, COLOR_CHARGING, 0);
    lv_obj_set_style_text_font(wf->charging_label, &lv_font_montserrat_14, 0);
    lv_obj_align(wf->charging_label, LV_ALIGN_TOP_MID, 0, 10);
    
    return wf;
}

// ============================================================================
// Update Functions
// ============================================================================

void pilot_watchface_set_time(pilot_watchface_t *wf, int hour, int minute, int second) {
    if (!wf) return;
    
    // Calculate angles
    // Hour: 360/12 = 30 degrees per hour, plus 0.5 degrees per minute
    float hour_angle = (hour % 12) * 30.0f + minute * 0.5f;
    
    // Minute: 360/60 = 6 degrees per minute, plus 0.1 degrees per second
    float minute_angle = minute * 6.0f + second * 0.1f;
    
    // Second: 360/60 = 6 degrees per second
    float second_angle = second * 6.0f;
    
    // Update hour hand
    static lv_point_precise_t hour_pts[2];
    hour_pts[0].x = DIAL_CENTER_X;
    hour_pts[0].y = DIAL_CENTER_Y;
    point_on_circle(DIAL_CENTER_X, DIAL_CENTER_Y, HOUR_HAND_LENGTH, hour_angle,
                    (int*)&hour_pts[1].x, (int*)&hour_pts[1].y);
    lv_line_set_points(wf->hour_hand, hour_pts, 2);
    
    // Update minute hand
    static lv_point_precise_t minute_pts[2];
    minute_pts[0].x = DIAL_CENTER_X;
    minute_pts[0].y = DIAL_CENTER_Y;
    point_on_circle(DIAL_CENTER_X, DIAL_CENTER_Y, MINUTE_HAND_LENGTH, minute_angle,
                    (int*)&minute_pts[1].x, (int*)&minute_pts[1].y);
    lv_line_set_points(wf->minute_hand, minute_pts, 2);
    
    // Update second hand (with small tail)
    static lv_point_precise_t second_pts[2];
    point_on_circle(DIAL_CENTER_X, DIAL_CENTER_Y, 20, second_angle + 180.0f,
                    (int*)&second_pts[0].x, (int*)&second_pts[0].y);  // Tail
    point_on_circle(DIAL_CENTER_X, DIAL_CENTER_Y, SECOND_HAND_LENGTH, second_angle,
                    (int*)&second_pts[1].x, (int*)&second_pts[1].y);  // Tip
    lv_line_set_points(wf->second_hand, second_pts, 2);
}

void pilot_watchface_set_date(pilot_watchface_t *wf, const char *day_name, int day_num) {
    if (!wf) return;
    
    lv_label_set_text(wf->day_label, day_name);
    
    char date_buf[4];
    snprintf(date_buf, sizeof(date_buf), "%02d", day_num);
    lv_label_set_text(wf->date_label, date_buf);
}

void pilot_watchface_set_accel(pilot_watchface_t *wf, float x, float y, float z) {
    if (!wf) return;
    
    // Clamp values to ±20 m/s² for display
    float max_val = 20.0f;
    x = (x > max_val) ? max_val : ((x < -max_val) ? -max_val : x);
    y = (y > max_val) ? max_val : ((y < -max_val) ? -max_val : y);
    z = (z > max_val) ? max_val : ((z < -max_val) ? -max_val : z);
    
    // Update value labels
    char buf[8];
    snprintf(buf, sizeof(buf), "%.1f", x);
    lv_label_set_text(wf->subdial_x_value, buf);
    
    snprintf(buf, sizeof(buf), "%.1f", y);
    lv_label_set_text(wf->subdial_y_value, buf);
    
    snprintf(buf, sizeof(buf), "%.1f", z);
    lv_label_set_text(wf->subdial_z_value, buf);
    
    // Update needle angles (-20 to +20 m/s² maps to -135 to +135 degrees)
    // Needle points up (0 degrees) when value is 0
    float x_angle = (x / max_val) * 135.0f;
    float y_angle = (y / max_val) * 135.0f;
    float z_angle = (z / max_val) * 135.0f;
    
    // Update X needle
    static lv_point_precise_t x_needle_pts[2];
    x_needle_pts[0].x = SUBDIAL_DIAMETER / 2;
    x_needle_pts[0].y = SUBDIAL_DIAMETER / 2;
    int nx, ny;
    point_on_circle(SUBDIAL_DIAMETER/2, SUBDIAL_DIAMETER/2, SUBDIAL_DIAMETER/2 - 12, x_angle, &nx, &ny);
    x_needle_pts[1].x = nx;
    x_needle_pts[1].y = ny;
    lv_line_set_points(wf->subdial_x_needle, x_needle_pts, 2);
    
    // Update Y needle
    static lv_point_precise_t y_needle_pts[2];
    y_needle_pts[0].x = SUBDIAL_DIAMETER / 2;
    y_needle_pts[0].y = SUBDIAL_DIAMETER / 2;
    point_on_circle(SUBDIAL_DIAMETER/2, SUBDIAL_DIAMETER/2, SUBDIAL_DIAMETER/2 - 12, y_angle, &nx, &ny);
    y_needle_pts[1].x = nx;
    y_needle_pts[1].y = ny;
    lv_line_set_points(wf->subdial_y_needle, y_needle_pts, 2);
    
    // Update Z needle
    static lv_point_precise_t z_needle_pts[2];
    z_needle_pts[0].x = SUBDIAL_DIAMETER / 2;
    z_needle_pts[0].y = SUBDIAL_DIAMETER / 2;
    point_on_circle(SUBDIAL_DIAMETER/2, SUBDIAL_DIAMETER/2, SUBDIAL_DIAMETER/2 - 12, z_angle, &nx, &ny);
    z_needle_pts[1].x = nx;
    z_needle_pts[1].y = ny;
    lv_line_set_points(wf->subdial_z_needle, z_needle_pts, 2);
}

void pilot_watchface_set_charging(pilot_watchface_t *wf, bool is_charging, int battery_percent) {
    if (!wf) return;
    
    if (is_charging) {
        char buf[32];
        snprintf(buf, sizeof(buf), LV_SYMBOL_CHARGE " %d%% Charging", battery_percent);
        lv_label_set_text(wf->charging_label, buf);
        lv_obj_set_style_text_color(wf->charging_label, COLOR_CHARGING, 0);
    } else {
        lv_label_set_text(wf->charging_label, "");  // Hide when not charging
    }
}

void pilot_watchface_destroy(pilot_watchface_t *wf) {
    if (!wf) return;

    // Delete all LVGL objects explicitly (parent screen is reused, not deleted)

    // Delete clock hands and center cap
    if (wf->center_cap) lv_obj_del(wf->center_cap);
    if (wf->second_hand) lv_obj_del(wf->second_hand);
    if (wf->minute_hand) lv_obj_del(wf->minute_hand);
    if (wf->hour_hand) lv_obj_del(wf->hour_hand);

    // Delete charging label
    if (wf->charging_label) lv_obj_del(wf->charging_label);

    // Delete date window (deletes children: day_label, date_label)
    if (wf->date_window) lv_obj_del(wf->date_window);

    // Delete sub-dials (deletes children: labels, values, needles)
    if (wf->subdial_x) lv_obj_del(wf->subdial_x);
    if (wf->subdial_y) lv_obj_del(wf->subdial_y);
    if (wf->subdial_z) lv_obj_del(wf->subdial_z);

    // Delete minute ticks
    for (int i = 0; i < 60; i++) {
        if (wf->minute_ticks[i]) lv_obj_del(wf->minute_ticks[i]);
    }

    // Delete hour markers and labels
    for (int i = 0; i < 12; i++) {
        if (wf->hour_markers[i]) lv_obj_del(wf->hour_markers[i]);
        if (wf->hour_labels[i]) lv_obj_del(wf->hour_labels[i]);
    }

    // Delete dial background
    if (wf->dial_bg) lv_obj_del(wf->dial_bg);

    // Free structure
    free(wf);
}
