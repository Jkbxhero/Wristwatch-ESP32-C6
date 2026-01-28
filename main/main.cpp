// Include qmi8658.h early - it defines M_PI and must come before <cmath>
// to avoid GCC "macro redefined" warnings (GCC has no flag to silence this)
#include "qmi8658.h"

#include <stdio.h>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_memory_utils.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif_sntp.h"
#include "esp_sntp.h"

// Bluetooth (NimBLE) includes - ESP32-C6 only supports BLE
#if CONFIG_BT_ENABLED
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#endif

#include "lvgl.h"

#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "pcf85063a.h"
#include "esp_pm.h"

// Watchface includes (both faces for dynamic selection)
#include "simple_watchface.h"
#include "pilot_watchface.h"

// Touch handling includes
#include "esp_lcd_touch.h"
#include "esp_lvgl_port.h"

// Spectrum analyzer
extern "C" {
#include "Spec_Analyzer.h"
}

// Apps
extern "C" {
#include "time_settings.h"
}

static const char *TAG = "Smartwatch";

// ============================================================================
// DISPLAY REINITIALIZATION - REFACTORED TO USE BSP METHODS
// ============================================================================

// Simplified structure to extract panel_handle ONCE during init
typedef struct {
    int disp_type;
    esp_lcd_panel_io_handle_t io_handle;
    esp_lcd_panel_handle_t panel_handle;
} lvgl_port_display_ctx_partial_t;

static esp_lcd_panel_handle_t g_panel_handle = NULL;

// Extract panel handle ONCE at startup (structure hack isolated here)
static esp_err_t store_panel_handle_from_lvgl(void)
{
    lv_display_t *disp = lv_display_get_default();
    if (!disp) {
        ESP_LOGE("PANEL", "No default display");
        return ESP_ERR_INVALID_STATE;
    }

    lvgl_port_display_ctx_partial_t *ctx = 
        (lvgl_port_display_ctx_partial_t *)lv_display_get_driver_data(disp);
    if (!ctx || !ctx->panel_handle) {
        ESP_LOGE("PANEL", "No panel handle in display context");
        return ESP_ERR_INVALID_STATE;
    }

    g_panel_handle = ctx->panel_handle;
    ESP_LOGI("PANEL", "Panel handle stored: %p", g_panel_handle);
    return ESP_OK;
}

// Reinitialize display using BSP methods (no structure hacking!)
static esp_err_t display_reinit_direct(void)
{
    if (!g_panel_handle) {
        ESP_LOGE("REINIT", "Panel handle not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI("REINIT", "Reinitializing display...");

    esp_err_t ret = esp_lcd_panel_reset(g_panel_handle);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));

    ret = esp_lcd_panel_init(g_panel_handle);
    if (ret != ESP_OK) return ret;

    esp_lcd_panel_set_gap(g_panel_handle, 0x16, 0);
    esp_lcd_panel_disp_on_off(g_panel_handle, true);

    ESP_LOGI("REINIT", "Display reinitialized successfully");
    return ESP_OK;
}

// Alias for display reinit (called from sleep management task)
static esp_err_t display_reinit_from_lvgl(void)
{
    return display_reinit_direct();
}

// ============================================================================
// END DISPLAY REINITIALIZATION REFACTOR
// ============================================================================

// ============================================================================
// START DECLARATIONS
// ============================================================================

// State machine flags
static bool startup_complete = false;
static bool wifi_init_complete = false;

// Active watchface (void* to support multiple watchface types)
static void *watchface = NULL;

// Sensor devices
static pcf85063a_dev_t rtc_dev = {};
static qmi8658_dev_t imu_dev = {};
static pcf85063a_datetime_t current_time = {};

// Battery widget components (graphical battery icon)
static lv_obj_t *battery_container = NULL;  // Container for entire battery widget
static lv_obj_t *battery_body = NULL;       // Main battery body
static lv_obj_t *battery_tip = NULL;        // Battery terminal/tip
static lv_obj_t *battery_fill = NULL;       // Fill level indicator
static lv_obj_t *battery_percent_label = NULL;  // Percentage text
static lv_obj_t *usb_icon = NULL;           // USB plugged in icon (shows when VBUS connected)
static lv_obj_t *charging_icon = NULL;      // Charging icon (shows when charging)

//WiFi setup, widgets and event handlers
#include "credentials.h"  // WiFi credentials (see credentials_template.h)
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;
static bool ntp_synced = false;
static bool wifi_is_running = false;    // Track WiFi state
static bool wifi_shutting_down = false; // Prevent reconnect during shutdown

// Task handles
static TaskHandle_t pmu_task_handle = NULL;
static TaskHandle_t wifi_task_handle = NULL;
static TaskHandle_t time_display_task_handle = NULL;

// Sleep management
static uint32_t inactivity_counter = 0;
static bool backlight_on = true;
static bool display_dimmed = false;  // True when in dim stage (for IMU/touch wake)
static uint8_t current_sleep_stage = 0;  // 0=active, 1=dim, 2=off, 3=ALDO2 off

// Sleep stage thresholds (in seconds)
#define STAGE_DIM_THRESHOLD      20  // Dim to 10% at 20s
#define STAGE_OFF_THRESHOLD      30  // Turn off backlight at 30s + destroy UI
#define STAGE_SLEEP_THRESHOLD    40  // Power off display (ALDO2) at 40s

// Twist-to-wake gesture configuration (clockwise then counter-clockwise wrist rotation)
#define TWIST_THRESHOLD 75.0f      // Minimum rotation speed to count as twist (deg/s)
#define TWIST_TIMEOUT_MS 1000      // Must complete twist within 1 second

// I2C timeout for PMU operations
#define I2C_MASTER_TIMEOUT_MS 100

// Touch wake detection
static volatile bool touch_detected_flag = false;

// Watchface selection
static int current_watchface_type = 0;      // 0=simple, 1=pilot (saved to NVS)
static bool on_settings_screen = false;     // True when showing watchface selector
#define WATCHFACE_TYPE_SIMPLE  0
#define WATCHFACE_TYPE_PILOT   1
#define WATCHFACE_COUNT        2
static const char *watchface_names = "Simple\nPilot";  // For roller widget

// ============================================================================
// MULTI-SCREEN NAVIGATION
// ============================================================================
typedef enum {
    SCREEN_WATCHFACE = 0,   // Default - watchface
    SCREEN_SCANNER,         // WiFi Scanner placeholder
    SCREEN_AUDIO,           // Audio codec placeholder
    SCREEN_APPS             // Apps page (swipe right from watchface)
} screen_state_t;

static screen_state_t current_screen = SCREEN_WATCHFACE;
static volatile bool imu_updates_paused = false;  // Pause sub-dial updates on app screens

// Swipe gesture detection
#define SWIPE_THRESHOLD_PX 80  // Minimum pixels for valid swipe
static lv_point_t swipe_start_point = {0, 0};
static bool tracking_swipe = false;

// Scanner screen UI elements
static lv_obj_t *scanner_container = NULL;
static lv_obj_t *scanner_return_btn = NULL;
static lv_obj_t *scanner_tabview = NULL;
static lv_obj_t *scanner_wifi_tab = NULL;
static lv_obj_t *scanner_bt_tab = NULL;
static lv_obj_t *scanner_espnow_tab = NULL;
static lv_obj_t *scanner_wifi_list = NULL;
static lv_obj_t *scanner_bt_list = NULL;
static lv_obj_t *scanner_espnow_list = NULL;
static lv_obj_t *scanner_wifi_status = NULL;
static lv_obj_t *scanner_bt_status = NULL;
static lv_obj_t *scanner_espnow_status = NULL;
static TaskHandle_t scanner_task_handle = NULL;
static bool scanner_wifi_running = false;
static uint8_t scanner_active_tab = 0;  // 0=WiFi, 1=BT, 2=ESP-NOW
static bool scanner_switching_tabs = false;
static bool scanner_screen_active = false;  // True when scanner screen exists, prevents UI updates during destruction

// WiFi scan results storage
#define MAX_SCAN_RESULTS 15
typedef struct {
    char ssid[33];
    int8_t rssi;
    wifi_auth_mode_t authmode;
} wifi_scan_result_t;
static wifi_scan_result_t scan_results[MAX_SCAN_RESULTS];
static uint16_t scan_result_count = 0;

// BLE scan results storage
#if CONFIG_BT_ENABLED
#define MAX_BT_SCAN_RESULTS 15
typedef struct {
    uint8_t addr[6];
    uint8_t addr_type;
    int8_t rssi;
    char name[32];
    bool has_name;
} bt_scan_result_t;
static bt_scan_result_t bt_scan_results[MAX_BT_SCAN_RESULTS];
static uint16_t bt_scan_result_count = 0;
static bool scanner_bt_running = false;
static bool bt_scan_complete = false;
static volatile bool bt_host_ready = false;
#endif

// ESP-NOW scan results storage (promiscuous mode detection)
#define MAX_ESPNOW_SCAN_RESULTS 15
typedef struct {
    uint8_t addr[6];
    int8_t rssi;
    uint32_t last_seen;     // Timestamp in ms
    uint16_t packet_count;  // Number of packets seen
} espnow_scan_result_t;
static espnow_scan_result_t espnow_scan_results[MAX_ESPNOW_SCAN_RESULTS];
static uint16_t espnow_scan_result_count = 0;
static bool scanner_espnow_running = false;
static volatile bool espnow_scan_active = false;

// Audio screen UI elements
static lv_obj_t *audio_container = NULL;
static lv_obj_t *audio_return_btn = NULL;

// Apps screen UI elements
static lv_obj_t *apps_container = NULL;
static lv_obj_t *apps_return_btn = NULL;
static lv_obj_t *apps_clock_btn = NULL;  // Clock icon button for time settings

// Time settings app state
static time_settings_t *time_settings_screen = NULL;
static bool on_time_settings_screen = false;

// Init screen UI elements (shown during WiFi/NTP startup)
static lv_obj_t *init_screen_label = NULL;
static bool init_screen_active = false;

// Forward declarations for navigation
static void transition_to_scanner(void);
static void transition_to_audio(void);
static void transition_to_apps(void);
static void nav_return_to_watchface(void);
static void create_scanner_screen(void);
static void destroy_scanner_screen(void);
static void create_audio_screen(void);
static void destroy_audio_screen(void);
static void create_apps_screen(void);
static void destroy_apps_screen(void);
static void destroy_watchface_only(void);
static void move_battery_to_foreground(void);
static void global_event_cb(lv_event_t *e);
static void create_init_screen(void);
static void destroy_init_screen(void);
static void update_init_screen_status(const char *status);
static void create_ui(void);

// Forward declarations for WiFi/radio management
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void wifi_shutdown(void);
static void wifi_scanner_shutdown(void);
static void espnow_scanner_shutdown(void);

// ============================================================================
// END DECLARATIONS
// ============================================================================


// ============================================================================
// MEMORY STATISTICS NVS FUNCTIONS - REMOVE FROM FINAL VERSION
// ============================================================================
#if ENABLE_MEMORY_STATS_NVS

// Load and display previous boot's memory statistics from NVS
static void load_previous_memory_stats(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(MEMORY_STATS_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No previous memory stats found (first boot)");
        return;
    }

    memory_stats_t stats = {0};
    size_t required_size = sizeof(memory_stats_t);
    err = nvs_get_blob(nvs_handle, "stats", &stats, &required_size);
    nvs_close(nvs_handle);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load previous memory stats");
        return;
    }

    ESP_LOGI(TAG, "========== PREVIOUS BOOT MEMORY STATS (Boot #%lu) ==========", stats.boot_count);
    ESP_LOGI(TAG, "Heap - Free: %zu KB, Used: %zu KB, Total: %zu KB",
             stats.free_heap / 1024, (stats.total_heap - stats.free_heap) / 1024, stats.total_heap / 1024);
    ESP_LOGI(TAG, "Heap - Min Free Ever: %zu KB (%.1f%% used at peak)",
             stats.min_free_heap / 1024, 100.0f * (stats.total_heap - stats.min_free_heap) / stats.total_heap);
    ESP_LOGI(TAG, "Heap - Largest Free Block: %zu KB", stats.largest_block / 1024);
    ESP_LOGI(TAG, "Stack - time_display: %lu bytes free (%.1f%% used)",
             stats.time_display_stack, 100.0f * (3072 - stats.time_display_stack) / 3072);
    ESP_LOGI(TAG, "Stack - pmu_handler: %lu bytes free (%.1f%% used)",
             stats.pmu_stack, 100.0f * (4096 - stats.pmu_stack) / 4096);
    ESP_LOGI(TAG, "Stack - imu_wake: %lu bytes free (%.1f%% used)",
             stats.imu_stack, 100.0f * (3072 - stats.imu_stack) / 3072);
    ESP_LOGI(TAG, "Stack - sleep_mgmt: %lu bytes free (%.1f%% used)",
             stats.sleep_stack, 100.0f * (3072 - stats.sleep_stack) / 3072);
    ESP_LOGI(TAG, "Stack - LVGL task: %lu bytes free (%.1f%% used)",
             stats.lvgl_stack, 100.0f * (8192 - stats.lvgl_stack) / 8192);
    ESP_LOGI(TAG, "================================================================");
}

// Save current memory statistics to NVS
static void save_memory_stats_to_nvs(void)
{
    memory_stats_t stats = {0};

    // Collect heap stats
    stats.free_heap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    stats.total_heap = heap_caps_get_total_size(MALLOC_CAP_8BIT);
    stats.min_free_heap = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
    stats.largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);

    // Collect stack high water marks
    if (time_display_task_handle) {
        stats.time_display_stack = uxTaskGetStackHighWaterMark(time_display_task_handle) * sizeof(StackType_t);
    }
    if (pmu_task_handle) {
        stats.pmu_stack = uxTaskGetStackHighWaterMark(pmu_task_handle) * sizeof(StackType_t);
    }
    TaskHandle_t imu_handle = xTaskGetHandle("imu_wake");
    if (imu_handle) {
        stats.imu_stack = uxTaskGetStackHighWaterMark(imu_handle) * sizeof(StackType_t);
    }
    TaskHandle_t sleep_handle = xTaskGetHandle("sleep_mgmt");
    if (sleep_handle) {
        stats.sleep_stack = uxTaskGetStackHighWaterMark(sleep_handle) * sizeof(StackType_t);
    }
    TaskHandle_t lvgl_handle = xTaskGetHandle("LVGL task");
    if (lvgl_handle) {
        stats.lvgl_stack = uxTaskGetStackHighWaterMark(lvgl_handle) * sizeof(StackType_t);
    }

    // Increment boot counter
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(MEMORY_STATS_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        memory_stats_t old_stats = {0};
        size_t required_size = sizeof(memory_stats_t);
        nvs_get_blob(nvs_handle, "stats", &old_stats, &required_size);
        stats.boot_count = old_stats.boot_count + 1;

        // Save new stats
        nvs_set_blob(nvs_handle, "stats", &stats, sizeof(memory_stats_t));
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "Memory stats saved to NVS (boot #%lu)", stats.boot_count);
    } else {
        ESP_LOGW(TAG, "Failed to save memory stats to NVS");
    }
}

#endif // ENABLE_MEMORY_STATS_NVS

// Print detailed memory and stack usage statistics
static void print_memory_stats(void)
{
    // Heap statistics
    size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_8BIT);
    size_t min_free_heap = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
    size_t largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);

    ESP_LOGI(TAG, "========== CURRENT MEMORY STATISTICS ==========");
    ESP_LOGI(TAG, "Heap - Free: %zu KB, Used: %zu KB, Total: %zu KB",
             free_heap / 1024, (total_heap - free_heap) / 1024, total_heap / 1024);
    ESP_LOGI(TAG, "Heap - Min Free Ever: %zu KB (%.1f%% used at peak)",
             min_free_heap / 1024, 100.0f * (total_heap - min_free_heap) / total_heap);
    ESP_LOGI(TAG, "Heap - Largest Free Block: %zu KB", largest_block / 1024);

    // Task stack usage (lower high water mark = more stack used)
    ESP_LOGI(TAG, "========== TASK STACK USAGE ==========");

    if (time_display_task_handle) {
        UBaseType_t hwm = uxTaskGetStackHighWaterMark(time_display_task_handle);
        ESP_LOGI(TAG, "time_display: %u bytes free (1948 allocated, %.1f%% used)",
                 hwm * sizeof(StackType_t), 100.0f * (1948 - hwm * sizeof(StackType_t)) / 1948);
    }

    if (pmu_task_handle) {
        UBaseType_t hwm = uxTaskGetStackHighWaterMark(pmu_task_handle);
        ESP_LOGI(TAG, "pmu_handler:  %u bytes free (2400 allocated, %.1f%% used)",
                 hwm * sizeof(StackType_t), 100.0f * (2400 - hwm * sizeof(StackType_t)) / 2400);
    }

    // IMU task (no handle saved, get by name)
    TaskHandle_t imu_handle = xTaskGetHandle("imu_wake");
    if (imu_handle) {
        UBaseType_t hwm = uxTaskGetStackHighWaterMark(imu_handle);
        ESP_LOGI(TAG, "imu_wake:     %u bytes free (2460 allocated, %.1f%% used)",
                 hwm * sizeof(StackType_t), 100.0f * (2460 - hwm * sizeof(StackType_t)) / 2460);
    }

    // Sleep management task
    TaskHandle_t sleep_handle = xTaskGetHandle("sleep_mgmt");
    if (sleep_handle) {
        UBaseType_t hwm = uxTaskGetStackHighWaterMark(sleep_handle);
        ESP_LOGI(TAG, "sleep_mgmt:   %u bytes free (2560 allocated, %.1f%% used)",
                 hwm * sizeof(StackType_t), 100.0f * (2560 - hwm * sizeof(StackType_t)) / 2560);
    }

    // LVGL task
    TaskHandle_t lvgl_handle = xTaskGetHandle("LVGL task");
    if (lvgl_handle) {
        UBaseType_t hwm = uxTaskGetStackHighWaterMark(lvgl_handle);
        ESP_LOGI(TAG, "LVGL task:    %u bytes free (8192 allocated, %.1f%% used)",
                 hwm * sizeof(StackType_t), 100.0f * (8192 - hwm * sizeof(StackType_t)) / 8192);
    }

    ESP_LOGI(TAG, "===============================================");

    // ========================================================================
    // SAVE STATS TO NVS - COMMENT OUT THIS BLOCK TO DISABLE
    // ========================================================================
    #if ENABLE_MEMORY_STATS_NVS
    save_memory_stats_to_nvs();
    #endif
    // ========================================================================
    // END SAVE STATS TO NVS
    // ========================================================================
}
// ============================================================================
// END MEMORY STATISTICS NVS FUNCTIONS
// ============================================================================

// ============================================================================
// START WATCHFACE NVS FUNCTIONS
// ============================================================================

#define WATCHFACE_NVS_NAMESPACE "watchface"
#define WATCHFACE_NVS_KEY "type"

// Load watchface type from NVS (returns default 0 if not found)
static int load_watchface_type_from_nvs(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(WATCHFACE_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No saved watchface preference (first boot), using default");
        return WATCHFACE_TYPE_SIMPLE;
    }

    int32_t saved_type = WATCHFACE_TYPE_SIMPLE;
    err = nvs_get_i32(nvs_handle, WATCHFACE_NVS_KEY, &saved_type);
    nvs_close(nvs_handle);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read watchface type from NVS");
        return WATCHFACE_TYPE_SIMPLE;
    }

    // Validate saved value
    if (saved_type < 0 || saved_type >= WATCHFACE_COUNT) {
        ESP_LOGW(TAG, "Invalid watchface type %ld in NVS, using default", saved_type);
        return WATCHFACE_TYPE_SIMPLE;
    }

    ESP_LOGI(TAG, "Loaded watchface type %ld from NVS", saved_type);
    return (int)saved_type;
}

// Save watchface type to NVS
static void save_watchface_type_to_nvs(int type)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(WATCHFACE_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for watchface: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_i32(nvs_handle, WATCHFACE_NVS_KEY, (int32_t)type);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save watchface type to NVS: %s", esp_err_to_name(err));
    } else {
        nvs_commit(nvs_handle);
        ESP_LOGI(TAG, "Saved watchface type %d to NVS", type);
    }
    nvs_close(nvs_handle);
}

// ============================================================================
// END WATCHFACE NVS FUNCTIONS
// ============================================================================

// ============================================================================
// START TIME MODE NVS FUNCTIONS (Auto/Manual setting)
// ============================================================================

#define TIME_MODE_NVS_NAMESPACE "timemode"
#define TIME_MODE_NVS_KEY "manual"

// Global time mode setting (loaded from NVS at startup)
static bool time_mode_is_manual = false;

// Load time mode from NVS (returns false/auto if not found)
static bool load_time_mode_from_nvs(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(TIME_MODE_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No saved time mode preference (first boot), using Auto");
        return false;  // Default to auto mode
    }

    int32_t saved_mode = 0;
    err = nvs_get_i32(nvs_handle, TIME_MODE_NVS_KEY, &saved_mode);
    nvs_close(nvs_handle);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read time mode from NVS");
        return false;
    }

    ESP_LOGI(TAG, "Loaded time mode from NVS: %s", saved_mode ? "Manual" : "Auto");
    return (bool)saved_mode;
}

// Save time mode to NVS
static void save_time_mode_to_nvs(bool is_manual)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(TIME_MODE_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for time mode: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_i32(nvs_handle, TIME_MODE_NVS_KEY, (int32_t)is_manual);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save time mode to NVS: %s", esp_err_to_name(err));
    } else {
        nvs_commit(nvs_handle);
        ESP_LOGI(TAG, "Saved time mode to NVS: %s", is_manual ? "Manual" : "Auto");
    }
    nvs_close(nvs_handle);
}

// ============================================================================
// END TIME MODE NVS FUNCTIONS
// ============================================================================

// ============================================================================
// START PMU DISPLAY AND AUDIO POWER CONTROL FUNCTIONS
// ============================================================================

static i2c_master_dev_handle_t pmu_dev_handle = NULL;

// PMU function declarations
typedef struct {
    int battery_percent;
    int battery_voltage_mv;
    int vbus_voltage_mv;
    bool is_charging;
    bool is_vbus_connected;
    float temperature_c;
} battery_info_t;

extern esp_err_t pmu_init();
extern void pmu_isr_handler(bool verbose);
extern battery_info_t get_battery_info();
extern bool pmu_check_vbus_changed();

// Display power control (ALDO2 - DSI PWR EN)
extern void pmu_display_power_off(void);
extern void pmu_display_power_on(void);
extern bool pmu_is_display_power_on(void);

// Audio power control (ALDO1 - A3V3)
extern void pmu_audio_power_off(void);
extern void pmu_audio_power_on(void);
extern bool pmu_is_audio_power_on(void);

// Forward declarations for UI management (defined later in file)
static void create_ui(void);
static void destroy_ui(void);
static void update_time_display(void);
static void update_battery_display(void);

// Initialize power management (frequency scaling only - light sleep breaks USB serial)
static void init_power_management(void)
{
#ifdef CONFIG_PM_ENABLE
    ESP_LOGI(TAG, "Configuring power management...");
    esp_pm_config_t pm_config = {
        .max_freq_mhz = 160,  // Max CPU frequency
        .min_freq_mhz = 10,   // Min CPU frequency when idle (enables freq scaling)
        .light_sleep_enable = false  // DISABLED - light sleep breaks USB serial on ESP32-C6
    };
    esp_err_t ret = esp_pm_configure(&pm_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Power management enabled: 160MHz max, 10MHz min, light sleep OFF");
    } else {
        ESP_LOGW(TAG, "Power management config failed: %s", esp_err_to_name(ret));
    }
#else
    ESP_LOGW(TAG, "Power management not enabled in sdkconfig (CONFIG_PM_ENABLE)");
#endif
}

// Initialize PMU device on shared I2C bus
esp_err_t pmu_i2c_init() {
    // Get the shared I2C bus handle from BSP (already initialized by RTC)
    i2c_master_bus_handle_t i2c_bus_handle = bsp_i2c_get_handle();
    if (i2c_bus_handle == NULL) {
        ESP_LOGE(TAG, "Failed to get I2C bus handle from BSP");
        return ESP_FAIL;
    }

    // Add PMU device to the existing bus
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x34,  // AXP2101 I2C address
        .scl_speed_hz = 400000,  // 400kHz
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = 0
        }
    };

    esp_err_t ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_config, &pmu_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add PMU device to I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

// PMU read function using new API
int pmu_register_read(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len) {
    esp_err_t ret = i2c_master_transmit_receive(pmu_dev_handle, &regAddr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PMU READ FAILED!");
        return -1;
    }
    return 0;
}

// PMU write function using new API
int pmu_register_write_byte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len) {
    uint8_t *buffer = (uint8_t *)malloc(len + 1);
    if (!buffer) return -1;
    buffer[0] = regAddr;
    memcpy(&buffer[1], data, len);

    esp_err_t ret = i2c_master_transmit(pmu_dev_handle, buffer, len + 1, I2C_MASTER_TIMEOUT_MS);
    free(buffer);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PMU WRITE FAILED!");
        return -1;
    }
    return 0;
}

// PMU monitoring task - checks VBUS changes every second, logs full status every 5 minutes
// Reduces polling frequency during sleep stages to save power
static void pmu_hander_task(void *args) {
    int loop_count = 0;
    const int VERBOSE_LOG_INTERVAL = 300;  // 300 seconds = 5 minutes

    while (1) {
        // Stage 3: Display powered off - minimal polling (every 30s just to check VBUS)
        if (current_sleep_stage >= 3) {
            vTaskDelay(pdMS_TO_TICKS(30000));  // 30 second polling - minimal activity
            if (pmu_check_vbus_changed()) {
                pmu_isr_handler(false);
            }
            continue;
        }

        // Stage 2: Display off - reduced polling (every 10s)
        if (current_sleep_stage >= 2) {
            vTaskDelay(pdMS_TO_TICKS(10000));  // 10 second polling
            if (pmu_check_vbus_changed()) {
                pmu_isr_handler(false);
            }
            continue;
        }

        // Normal operation: Check if VBUS changed (USB plugged/unplugged)
        if (pmu_check_vbus_changed()) {
            pmu_isr_handler(false);  // Log minimal info on VBUS change
        }

        // Verbose logging every 5 minutes
        loop_count++;
        if (loop_count >= VERBOSE_LOG_INTERVAL) {
            pmu_isr_handler(true);  // Verbose log
            print_memory_stats();    // Print memory and stack usage
            loop_count = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));  // Check every second
    }
}


// ============================================================================
// END PMU DISPLAY AND AUDIO POWER CONTROL FUNCTIONS
// ============================================================================

// ========================================================================
// BEGIN CONFIGURE INITIAL TIME FROM NTP SERVER - IMMEDIATELY SHUT DOWN WIFI AFTER
// ========================================================================

// Shutdown WiFi to save power
static void wifi_shutdown(void)
{
    if (wifi_is_running) {
        wifi_shutting_down = true;  // Prevent event handler reconnects
        ESP_LOGI(TAG, "Shutting down WiFi to save power...");
        esp_wifi_stop();
        vTaskDelay(pdMS_TO_TICKS(100));  // Let events drain

        // Unregister event handlers to prevent memory leak on reinit
        esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler);
        esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler);

        esp_wifi_deinit();

        // Allow WiFi hardware to fully release before next operation
        vTaskDelay(pdMS_TO_TICKS(100));

        wifi_is_running = false;
        wifi_shutting_down = false;
        ESP_LOGI(TAG, "WiFi shut down successfully");
    }
}

// NTP sync callback
static void ntp_sync_callback(struct timeval *tv)
{
    ESP_LOGI(TAG, "NTP time synced!");
    ntp_synced = true;

    // Get time from system and update RTC
    time_t now = tv->tv_sec;
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    pcf85063a_datetime_t rtc_time = {
        .year = (uint16_t)(timeinfo.tm_year + 1900),
        .month = (uint8_t)(timeinfo.tm_mon + 1),
        .day = (uint8_t)timeinfo.tm_mday,
        .dotw = (uint8_t)timeinfo.tm_wday,
        .hour = (uint8_t)timeinfo.tm_hour,
        .min = (uint8_t)timeinfo.tm_min,
        .sec = (uint8_t)timeinfo.tm_sec
    };

    if (pcf85063a_set_time_date(&rtc_dev, rtc_time) == ESP_OK) {
        ESP_LOGI(TAG, "RTC updated from NTP: %04d-%02d-%02d %02d:%02d:%02d",
                 rtc_time.year, rtc_time.month, rtc_time.day,
                 rtc_time.hour, rtc_time.min, rtc_time.sec);
    }
}

// Initialize NTP
static void ntp_init(void)
{
    ESP_LOGI(TAG, "Initializing NTP client...");

    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(NTP_SERVER);
    config.sync_cb = ntp_sync_callback;
    config.start = false;  // Don't start yet

    esp_err_t ret = esp_netif_sntp_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init NTP: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "NTP configured for server: %s", NTP_SERVER);
}

// Start NTP sync
static void ntp_start(void)
{
    ESP_LOGI(TAG, "Starting NTP sync...");
    esp_err_t ret = esp_netif_sntp_start();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to start NTP: %s", esp_err_to_name(ret));
    }
}

// Forward declarations
static bool wifi_init(void);  // Returns true on success, false on failure

// WiFi event handler
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        // Only auto-connect if not in shutdown/scanner mode
        if (!wifi_shutting_down && !scanner_wifi_running) {
            ESP_LOGI(TAG, "WiFi started, connecting...");
            esp_wifi_connect();
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // Don't retry if we're shutting down or in scanner mode
        if (wifi_shutting_down || scanner_wifi_running) {
            ESP_LOGI(TAG, "WiFi disconnected (shutdown in progress)");
            return;
        }
        if (s_retry_num < 5) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry connecting to AP, attempt %d", s_retry_num);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGI(TAG, "Failed to connect to AP");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

        printf("\n========================================\n");
        printf("✓ CONNECTED TO %s\n", WIFI_SSID);
        printf("IP Address: " IPSTR "\n", IP2STR(&event->ip_info.ip));
        printf("========================================\n\n");

        // Start NTP sync after WiFi connects
        ntp_start();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE) {
        ESP_LOGI(TAG, "WiFi scan done");
    }
}

// Startup sequencer task: manages WiFi initialization after system stabilizes
// After WiFi/NTP completes, creates the full watchface UI
static void startup_complete_task(void *pvParameters)
{
    ESP_LOGI(TAG, "=== STARTUP COMPLETE ===");

    // Declare variables at top to avoid goto crossing initialization
    int wait_count = 0;
    bool wifi_failed = false;

    // Wait 1 second after startup before starting WiFi
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Check if time mode is manual - skip WiFi/NTP entirely
    if (time_mode_is_manual) {
        ESP_LOGI(TAG, "Time mode is MANUAL - skipping WiFi/NTP sync");
        wifi_init_complete = true;  // Mark WiFi as done (wasn't needed)
        goto create_watchface;
    }

    // NOTE: Screen stays black during WiFi - no LVGL running yet to save RAM
    ESP_LOGI(TAG, "Time mode is AUTO - beginning WiFi/NTP sequence");

    // Initialize WiFi - if this fails, watch still works with RTC time
    ESP_LOGI(TAG, "Initializing WiFi for NTP sync...");
    if (!wifi_init()) {
        ESP_LOGW(TAG, "WiFi initialization failed - watch will use RTC time only");
        wifi_init_complete = true;

        // WiFi failed - create watchface immediately (no WiFi RAM to free)
        vTaskDelay(pdMS_TO_TICKS(500));
        goto create_watchface;
    }

    // Wait for NTP sync or WiFi failure (timeout after 60 seconds)
    while (!ntp_synced && wait_count < 60) {
        // Check if WiFi connection failed (after 5 retries)
        EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
        if (bits & WIFI_FAIL_BIT) {
            ESP_LOGW(TAG, "WiFi connection failed after retries, giving up on NTP");
            wifi_failed = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        wait_count++;
    }

    if (ntp_synced) {
        ESP_LOGI(TAG, "NTP synced successfully, shutting down WiFi...");
        vTaskDelay(pdMS_TO_TICKS(500));
        wifi_shutdown();
    } else if (wifi_failed) {
        ESP_LOGW(TAG, "WiFi connection failed, shutting down WiFi...");
        wifi_shutdown();
    } else {
        ESP_LOGI(TAG, "NTP sync timeout (%d seconds), shutting down WiFi anyway...", wait_count);
        wifi_shutdown();
    }

    // WiFi complete
    ESP_LOGI(TAG, "=== WIFI COMPLETE ===");
    wifi_init_complete = true;

create_watchface:
    // Now WiFi is shut down and ~40KB RAM is freed
    // Initialize LVGL display NOW (was delayed to save RAM for WiFi)
    ESP_LOGI(TAG, "WiFi shutdown complete, heap free: %lu - initializing display",
             esp_get_free_heap_size());

    // Initialize LVGL and display driver (this was delayed from app_main)
    bsp_display_start();
    ESP_LOGI(TAG, "Display started, heap free: %lu", esp_get_free_heap_size());

    // Store panel handle for display reinitialization after deep sleep
    esp_err_t panel_ret = store_panel_handle_from_lvgl();
    if (panel_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to store panel handle - display reinit after sleep may fail");
    }

    // Small delay for display to initialize
    vTaskDelay(pdMS_TO_TICKS(100));

    // Create the full watchface UI (now with plenty of heap)
    create_ui();

    // Turn on backlight now that watchface is rendered
    vTaskDelay(pdMS_TO_TICKS(200));
    bsp_display_backlight_on();

    // Update time display with RTC time
    update_time_display();
    update_battery_display();

    ESP_LOGI(TAG, "Watchface created, heap free: %lu", esp_get_free_heap_size());

    // Print memory statistics
    vTaskDelay(pdMS_TO_TICKS(1000));
    print_memory_stats();

    // Startup sequence complete, delete this task
    vTaskDelete(NULL);
}

// WiFi only used for NTP sync at startup
static void wifi_scan_task(void *pvParameters)
{
    vTaskDelete(NULL);  // Task not needed for watchface
}

// Returns true on success, false on failure (watch still works without WiFi)
static bool wifi_init(void)
{
    esp_err_t ret;
    
    // Only create event loop and netif on first call
    static bool first_init = true;
    if (first_init) {
        s_wifi_event_group = xEventGroupCreate();
        
        // Critical: Network interface initialization
        ret = esp_netif_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to init netif: %s", esp_err_to_name(ret));
            return false;
        }
        
        // Critical: Event loop for WiFi callbacks
        ret = esp_event_loop_create_default();
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            // ESP_ERR_INVALID_STATE means already created - that's OK
            ESP_LOGE(TAG, "Failed to create event loop: %s", esp_err_to_name(ret));
            return false;
        }
        
        esp_netif_create_default_wifi_sta();
        first_init = false;
    }

    // Critical: WiFi driver initialization
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi init failed: %s", esp_err_to_name(ret));
        return false;
    }

    // Register event handlers - these rarely fail
    ret = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                      &wifi_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to register WiFi event handler: %s", esp_err_to_name(ret));
        // Continue anyway - might still work
    }
    
    ret = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                      &wifi_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to register IP event handler: %s", esp_err_to_name(ret));
        // Continue anyway - might still work
    }

    // Configure WiFi credentials
    wifi_config_t wifi_config = {};
    memcpy(wifi_config.sta.ssid, WIFI_SSID, strlen(WIFI_SSID));
    memcpy(wifi_config.sta.password, WIFI_PASS, strlen(WIFI_PASS));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    // Set WiFi mode and config - these rarely fail with valid data
    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi config: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        return false;
    }

    // Disable WiFi power save to prevent TCP connection issues
    ret = esp_wifi_set_ps(WIFI_PS_NONE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to disable WiFi power save: %s", esp_err_to_name(ret));
        // Not critical - continue anyway
    } else {
        ESP_LOGI(TAG, "WiFi power save disabled");
    }

    wifi_is_running = true;
    ESP_LOGI(TAG, "WiFi initialized, connecting to %s...", WIFI_SSID);
    return true;
}

// ========================================================================
// WIFI SCANNER FUNCTIONS
// ========================================================================

// Initialize WiFi for scanning only (no connection)
// Note: netif and event loop were already created during startup NTP sequence
static bool wifi_init_for_scan(void)
{
    esp_err_t ret;

    // Guard: prevent double initialization
    if (scanner_wifi_running || scanner_espnow_running || wifi_is_running) {
        ESP_LOGW(TAG, "WiFi already running (wifi=%d, scanner=%d, espnow=%d), shutting down first",
                 wifi_is_running, scanner_wifi_running, scanner_espnow_running);
        wifi_scanner_shutdown();
        espnow_scanner_shutdown();
        wifi_shutdown();
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    ESP_LOGI(TAG, "WiFi init starting, heap free: %lu", esp_get_free_heap_size());

    // Reinitialize WiFi driver (was deinit'd after NTP sync to save RAM)
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Scanner: WiFi init failed: %s (heap: %lu)",
                 esp_err_to_name(ret), esp_get_free_heap_size());
        return false;
    }

    // Set to STA mode (required for scanning)
    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Scanner: Failed to set WiFi mode: %s", esp_err_to_name(ret));
        esp_wifi_deinit();  // Clean up on failure
        return false;
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Scanner: Failed to start WiFi: %s", esp_err_to_name(ret));
        esp_wifi_deinit();  // Clean up on failure
        return false;
    }

    scanner_wifi_running = true;
    ESP_LOGI(TAG, "WiFi initialized for scanning");
    return true;
}

// Shutdown WiFi scanner to free RAM
static void wifi_scanner_shutdown(void)
{
    if (scanner_wifi_running) {
        wifi_shutting_down = true;  // Prevent event handler reconnects
        ESP_LOGI(TAG, "Shutting down scanner WiFi...");
        esp_wifi_stop();
        vTaskDelay(pdMS_TO_TICKS(100));  // Let events drain
        esp_wifi_deinit();
        scanner_wifi_running = false;
        wifi_shutting_down = false;
        vTaskDelay(pdMS_TO_TICKS(500));  // Allow memory to be fully released
        ESP_LOGI(TAG, "Scanner WiFi shut down, heap: %lu", esp_get_free_heap_size());
    }
}

// Compare function for sorting by RSSI (strongest first)
static int compare_rssi(const void *a, const void *b)
{
    const wifi_scan_result_t *ap_a = (const wifi_scan_result_t *)a;
    const wifi_scan_result_t *ap_b = (const wifi_scan_result_t *)b;
    return ap_b->rssi - ap_a->rssi;  // Descending order (stronger signal first)
}

// Get signal strength indicator string
static const char* get_signal_bars(int8_t rssi)
{
    if (rssi >= -50) return LV_SYMBOL_WIFI " ====";      // Excellent
    if (rssi >= -60) return LV_SYMBOL_WIFI " === ";      // Good
    if (rssi >= -70) return LV_SYMBOL_WIFI " ==  ";      // Fair
    if (rssi >= -80) return LV_SYMBOL_WIFI " =   ";      // Weak
    return LV_SYMBOL_WIFI " .   ";                        // Very weak
}

// Get auth mode string
static const char* get_auth_string(wifi_auth_mode_t authmode)
{
    switch (authmode) {
        case WIFI_AUTH_OPEN:            return "Open";
        case WIFI_AUTH_WEP:             return "WEP";
        case WIFI_AUTH_WPA_PSK:         return "WPA";
        case WIFI_AUTH_WPA2_PSK:        return "WPA2";
        case WIFI_AUTH_WPA_WPA2_PSK:    return "WPA/2";
        case WIFI_AUTH_WPA3_PSK:        return "WPA3";
        case WIFI_AUTH_WPA2_WPA3_PSK:   return "WPA2/3";
        default:                        return "Secure";
    }
}

// Task to perform WiFi scan (WiFi must already be initialized before calling)
static void wifi_scanner_task(void *pvParameters)
{
    ESP_LOGI(TAG, "WiFi scanner task started");

    // Give WiFi time to stabilize
    vTaskDelay(pdMS_TO_TICKS(500));

    // Check if screen still active before UI update
    if (!scanner_screen_active) {
        scanner_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    // Update status
    bsp_display_lock(0);
    if (scanner_screen_active && scanner_wifi_status) {
        lv_label_set_text(scanner_wifi_status, "Scanning...");
    }
    bsp_display_unlock();

    // Configure scan parameters
    wifi_scan_config_t scan_config = {
        .ssid = NULL,           // Scan all SSIDs
        .bssid = NULL,          // Scan all BSSIDs
        .channel = 0,           // Scan all channels
        .show_hidden = true,    // Show hidden networks
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time = {
            .active = {
                .min = 100,
                .max = 300
            },
            .passive = 0
        },
        .home_chan_dwell_time = 0,
        .channel_bitmap = {
            .ghz_2_channels = 0,
            .ghz_5_channels = 0
        },
        .coex_background_scan = false
    };

    // Start scan (blocking)
    esp_err_t ret = esp_wifi_scan_start(&scan_config, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Scan failed: %s", esp_err_to_name(ret));
        bsp_display_lock(0);
        if (scanner_wifi_status) {
            lv_label_set_text(scanner_wifi_status, "Scan failed!");
        }
        bsp_display_unlock();
        scanner_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    // Get scan results count
    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    ESP_LOGI(TAG, "Found %d access points", ap_count);

    if (ap_count > MAX_SCAN_RESULTS) {
        ap_count = MAX_SCAN_RESULTS;
    }

    // Allocate memory for AP records
    wifi_ap_record_t *ap_records = (wifi_ap_record_t *)malloc(ap_count * sizeof(wifi_ap_record_t));
    if (ap_records == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for scan results");
        bsp_display_lock(0);
        if (scanner_wifi_status) {
            lv_label_set_text(scanner_wifi_status, "Memory error!");
        }
        bsp_display_unlock();
        scanner_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    ret = esp_wifi_scan_get_ap_records(&ap_count, ap_records);
    if (ret != ESP_OK) {
        free(ap_records);
        ESP_LOGE(TAG, "Failed to get scan records: %s", esp_err_to_name(ret));
        bsp_display_lock(0);
        if (scanner_wifi_status) {
            lv_label_set_text(scanner_wifi_status, "Read error!");
        }
        bsp_display_unlock();
        scanner_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    // Copy to our results array
    scan_result_count = ap_count;
    for (int i = 0; i < ap_count; i++) {
        strncpy(scan_results[i].ssid, (char *)ap_records[i].ssid, 32);
        scan_results[i].ssid[32] = '\0';
        scan_results[i].rssi = ap_records[i].rssi;
        scan_results[i].authmode = ap_records[i].authmode;
    }
    free(ap_records);

    // Sort by signal strength (strongest first)
    qsort(scan_results, scan_result_count, sizeof(wifi_scan_result_t), compare_rssi);

    // Check if screen still active before UI update
    if (!scanner_screen_active) {
        scanner_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    // Update UI with results
    bsp_display_lock(0);

    if (scanner_screen_active && scanner_wifi_status) {
        char status_text[32];
        snprintf(status_text, sizeof(status_text), "Found %d networks", scan_result_count);
        lv_label_set_text(scanner_wifi_status, status_text);
    }

    // Populate the list
    if (scanner_screen_active && scanner_wifi_list) {
        lv_obj_clean(scanner_wifi_list);  // Clear existing items

        for (int i = 0; i < scan_result_count; i++) {
            lv_obj_t *item = lv_obj_create(scanner_wifi_list);
            lv_obj_set_size(item, lv_pct(100), 42);
            lv_obj_set_style_bg_color(item, lv_color_hex(0x1a3a5c), 0);
            lv_obj_set_style_border_width(item, 0, 0);
            lv_obj_set_style_radius(item, 6, 0);
            lv_obj_set_style_pad_all(item, 6, 0);
            lv_obj_remove_flag(item, LV_OBJ_FLAG_SCROLLABLE);

            // SSID label
            lv_obj_t *ssid_label = lv_label_create(item);
            const char *display_ssid = strlen(scan_results[i].ssid) > 0 ?
                                       scan_results[i].ssid : "(Hidden)";
            lv_label_set_text(ssid_label, display_ssid);
            lv_obj_set_style_text_color(ssid_label, lv_color_white(), 0);
            lv_obj_set_style_text_font(ssid_label, &lv_font_montserrat_12, 0);
            lv_obj_align(ssid_label, LV_ALIGN_TOP_LEFT, 0, 0);

            // Signal strength + auth info
            lv_obj_t *info_label = lv_label_create(item);
            char info_text[48];
            snprintf(info_text, sizeof(info_text), "%s %ddBm %s",
                    get_signal_bars(scan_results[i].rssi),
                    scan_results[i].rssi,
                    get_auth_string(scan_results[i].authmode));
            lv_label_set_text(info_label, info_text);
            lv_obj_set_style_text_color(info_label, lv_color_hex(0x88aacc), 0);
            lv_obj_set_style_text_font(info_label, &lv_font_montserrat_10, 0);
            lv_obj_align(info_label, LV_ALIGN_BOTTOM_LEFT, 0, 0);
        }
    }

    bsp_display_unlock();

    ESP_LOGI(TAG, "WiFi scan complete, task exiting");
    scanner_task_handle = NULL;
    vTaskDelete(NULL);
}

// ========================================================================
// END WIFI SCANNER FUNCTIONS
// ========================================================================

// ========================================================================
// BLE SCANNER FUNCTIONS
// ========================================================================

#if CONFIG_BT_ENABLED

// Get BLE signal strength indicator string
static const char* get_bt_signal_bars(int8_t rssi)
{
    if (rssi >= -50) return LV_SYMBOL_BLUETOOTH " ====";   // Excellent
    if (rssi >= -65) return LV_SYMBOL_BLUETOOTH " === ";   // Good
    if (rssi >= -80) return LV_SYMBOL_BLUETOOTH " ==  ";   // Fair
    if (rssi >= -90) return LV_SYMBOL_BLUETOOTH " =   ";   // Weak
    return LV_SYMBOL_BLUETOOTH " .   ";                     // Very weak
}

// Compare function for sorting BLE results by RSSI (strongest first)
static int compare_bt_rssi(const void *a, const void *b)
{
    const bt_scan_result_t *dev_a = (const bt_scan_result_t *)a;
    const bt_scan_result_t *dev_b = (const bt_scan_result_t *)b;
    return dev_b->rssi - dev_a->rssi;
}

// Check if device already in results (by address)
static int find_bt_device(const uint8_t *addr)
{
    for (int i = 0; i < bt_scan_result_count; i++) {
        if (memcmp(bt_scan_results[i].addr, addr, 6) == 0) {
            return i;
        }
    }
    return -1;
}

// BLE GAP event handler - processes scan results
static int ble_gap_event_handler(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
        case BLE_GAP_EVENT_DISC: {
            // Found a device during scan
            const struct ble_gap_disc_desc *disc = &event->disc;

            // Check if already in list
            int idx = find_bt_device(disc->addr.val);

            if (idx >= 0) {
                // Update RSSI if stronger
                if (disc->rssi > bt_scan_results[idx].rssi) {
                    bt_scan_results[idx].rssi = disc->rssi;
                }
            } else if (bt_scan_result_count < MAX_BT_SCAN_RESULTS) {
                // Add new device
                idx = bt_scan_result_count++;
                memcpy(bt_scan_results[idx].addr, disc->addr.val, 6);
                bt_scan_results[idx].addr_type = disc->addr.type;
                bt_scan_results[idx].rssi = disc->rssi;
                bt_scan_results[idx].has_name = false;
                bt_scan_results[idx].name[0] = '\0';

                // Parse advertising data for name
                struct ble_hs_adv_fields fields;
                if (ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data) == 0) {
                    if (fields.name != NULL && fields.name_len > 0) {
                        int len = fields.name_len < 31 ? fields.name_len : 31;
                        memcpy(bt_scan_results[idx].name, fields.name, len);
                        bt_scan_results[idx].name[len] = '\0';
                        bt_scan_results[idx].has_name = true;
                    }
                }
            }
            break;
        }

        case BLE_GAP_EVENT_DISC_COMPLETE:
            ESP_LOGI(TAG, "BLE scan complete");
            bt_scan_complete = true;
            break;

        default:
            break;
    }
    return 0;
}

// Called when BLE host and controller sync
static void ble_on_sync(void)
{
    ESP_LOGI(TAG, "BLE host synced");
    bt_host_ready = true;
}

// Called on BLE host reset
static void ble_on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE host reset, reason=%d", reason);
    bt_host_ready = false;
}

// NimBLE host task
static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE host task started");
    nimble_port_run();  // This returns when nimble_port_stop() is called
    nimble_port_freertos_deinit();
}

// Initialize BLE for scanning
static bool ble_init_for_scan(void)
{
    esp_err_t ret;

    // BLE stack requires significant heap (~50-80KB)
    size_t free_heap = esp_get_free_heap_size();
    if (free_heap < 80000) {
        ESP_LOGE(TAG, "Insufficient heap for BLE: %u bytes (need ~80KB)", free_heap);
        return false;
    }
    ESP_LOGI(TAG, "BLE init starting, heap free: %u", free_heap);

    // Initialize NimBLE
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NimBLE init failed: %s", esp_err_to_name(ret));
        return false;
    }

    // Configure GAP
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.reset_cb = ble_on_reset;

    // Set device name
    ble_svc_gap_device_name_set("C6Watch");

    // Start the host task
    nimble_port_freertos_init(ble_host_task);

    // Wait for host to sync (max 3 seconds)
    int wait = 0;
    while (!bt_host_ready && wait < 30) {
        vTaskDelay(pdMS_TO_TICKS(100));
        wait++;
    }

    if (!bt_host_ready) {
        ESP_LOGE(TAG, "BLE host failed to sync");
        return false;
    }

    scanner_bt_running = true;
    ESP_LOGI(TAG, "BLE initialized for scanning");
    return true;
}

// Shutdown BLE scanner
static void ble_scanner_shutdown(void)
{
    if (scanner_bt_running) {
        ESP_LOGI(TAG, "Shutting down BLE scanner...");

        // Stop any ongoing scan
        ble_gap_disc_cancel();
        vTaskDelay(pdMS_TO_TICKS(100));

        // Stop NimBLE - this signals the host task to exit
        int ret = nimble_port_stop();
        if (ret == 0) {
            // Wait for host task to finish (it calls nimble_port_freertos_deinit)
            vTaskDelay(pdMS_TO_TICKS(1000));
            ret = nimble_port_deinit();
            ESP_LOGI(TAG, "nimble_port_deinit returned %d", ret);
        }

        // Give substantial time for BLE controller memory to be fully released
        vTaskDelay(pdMS_TO_TICKS(1500));

        scanner_bt_running = false;
        bt_host_ready = false;
        ESP_LOGI(TAG, "BLE scanner shut down, heap free: %lu", esp_get_free_heap_size());
    }
}

// Task to perform BLE scan (BLE must already be initialized before calling)
static void bt_scanner_task(void *pvParameters)
{
    ESP_LOGI(TAG, "BLE scanner task started");

    // Check if screen still active
    if (!scanner_screen_active) {
        scanner_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    // Clear previous results
    bt_scan_result_count = 0;
    bt_scan_complete = false;

    // Update status
    bsp_display_lock(0);
    if (scanner_screen_active && scanner_bt_status) {
        lv_label_set_text(scanner_bt_status, "Scanning for devices...");
    }
    bsp_display_unlock();

    // Configure scan parameters
    struct ble_gap_disc_params scan_params = {
        .itvl = 160,              // 100ms interval (160 * 0.625ms)
        .window = 80,             // 50ms window (80 * 0.625ms)
        .filter_policy = BLE_HCI_SCAN_FILT_NO_WL,
        .limited = 0,
        .passive = 0,             // Active scan (request scan responses)
        .filter_duplicates = 0,   // Report all advertisements
        .disable_observer_mode = 0,
    };

    // Start scanning (5 seconds)
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, 5000, &scan_params,
                          ble_gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE scan start failed: %d", rc);
        bsp_display_lock(0);
        if (scanner_bt_status) {
            lv_label_set_text(scanner_bt_status, "Scan start failed!");
        }
        bsp_display_unlock();
        scanner_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    // Wait for scan to complete (check screen active periodically)
    while (!bt_scan_complete && scanner_screen_active) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Check if screen was destroyed during scan
    if (!scanner_screen_active) {
        scanner_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Found %d BLE devices", bt_scan_result_count);

    // Sort by signal strength
    if (bt_scan_result_count > 1) {
        qsort(bt_scan_results, bt_scan_result_count, sizeof(bt_scan_result_t), compare_bt_rssi);
    }

    // Update UI with results
    bsp_display_lock(0);

    if (scanner_screen_active && scanner_bt_status) {
        char status_text[32];
        snprintf(status_text, sizeof(status_text), "Found %d devices", bt_scan_result_count);
        lv_label_set_text(scanner_bt_status, status_text);
    }

    // Populate the list
    if (scanner_screen_active && scanner_bt_list) {
        lv_obj_clean(scanner_bt_list);

        for (int i = 0; i < bt_scan_result_count; i++) {
            lv_obj_t *item = lv_obj_create(scanner_bt_list);
            lv_obj_set_size(item, lv_pct(100), 42);
            lv_obj_set_style_bg_color(item, lv_color_hex(0x1a3a5c), 0);
            lv_obj_set_style_border_width(item, 0, 0);
            lv_obj_set_style_radius(item, 6, 0);
            lv_obj_set_style_pad_all(item, 6, 0);
            lv_obj_remove_flag(item, LV_OBJ_FLAG_SCROLLABLE);

            // Device name or address
            lv_obj_t *name_label = lv_label_create(item);
            if (bt_scan_results[i].has_name && strlen(bt_scan_results[i].name) > 0) {
                lv_label_set_text(name_label, bt_scan_results[i].name);
            } else {
                char addr_str[18];
                snprintf(addr_str, sizeof(addr_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                        bt_scan_results[i].addr[5], bt_scan_results[i].addr[4],
                        bt_scan_results[i].addr[3], bt_scan_results[i].addr[2],
                        bt_scan_results[i].addr[1], bt_scan_results[i].addr[0]);
                lv_label_set_text(name_label, addr_str);
            }
            lv_obj_set_style_text_color(name_label, lv_color_white(), 0);
            lv_obj_set_style_text_font(name_label, &lv_font_montserrat_12, 0);
            lv_obj_align(name_label, LV_ALIGN_TOP_LEFT, 0, 0);

            // Signal strength info
            lv_obj_t *info_label = lv_label_create(item);
            char info_text[48];
            snprintf(info_text, sizeof(info_text), "%s %ddBm %s",
                    get_bt_signal_bars(bt_scan_results[i].rssi),
                    bt_scan_results[i].rssi,
                    bt_scan_results[i].addr_type == BLE_ADDR_PUBLIC ? "Public" : "Random");
            lv_label_set_text(info_label, info_text);
            lv_obj_set_style_text_color(info_label, lv_color_hex(0x88aacc), 0);
            lv_obj_set_style_text_font(info_label, &lv_font_montserrat_10, 0);
            lv_obj_align(info_label, LV_ALIGN_BOTTOM_LEFT, 0, 0);
        }
    }

    bsp_display_unlock();

    ESP_LOGI(TAG, "BLE scan complete, task exiting");
    scanner_task_handle = NULL;
    vTaskDelete(NULL);
}

#endif // CONFIG_BT_ENABLED

// ========================================================================
// END BLE SCANNER FUNCTIONS
// ========================================================================

// ========================================================================
// ESP-NOW SCANNER FUNCTIONS (Promiscuous Mode)
// ========================================================================

// ESP-NOW uses vendor-specific action frames
// OUI for Espressif: 0x18, 0xFE, 0x34
static const uint8_t ESPRESSIF_OUI[3] = {0x18, 0xFE, 0x34};

// Get ESP-NOW signal strength indicator
static const char* get_espnow_signal_bars(int8_t rssi)
{
    if (rssi >= -50) return LV_SYMBOL_SHUFFLE " ====";   // Excellent
    if (rssi >= -65) return LV_SYMBOL_SHUFFLE " === ";   // Good
    if (rssi >= -80) return LV_SYMBOL_SHUFFLE " ==  ";   // Fair
    if (rssi >= -90) return LV_SYMBOL_SHUFFLE " =   ";   // Weak
    return LV_SYMBOL_SHUFFLE " .   ";                     // Very weak
}

// Compare function for sorting ESP-NOW results by packet count (most active first)
static int compare_espnow_packets(const void *a, const void *b)
{
    const espnow_scan_result_t *dev_a = (const espnow_scan_result_t *)a;
    const espnow_scan_result_t *dev_b = (const espnow_scan_result_t *)b;
    return dev_b->packet_count - dev_a->packet_count;
}

// Find device in ESP-NOW results by MAC address
static int find_espnow_device(const uint8_t *addr)
{
    for (int i = 0; i < espnow_scan_result_count; i++) {
        if (memcmp(espnow_scan_results[i].addr, addr, 6) == 0) {
            return i;
        }
    }
    return -1;
}

// Promiscuous mode callback - called for every received packet
static void IRAM_ATTR espnow_promiscuous_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    if (!espnow_scan_active) return;

    // Only interested in management frames (action frames)
    if (type != WIFI_PKT_MGMT) return;

    const wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
    const uint8_t *payload = pkt->payload;
    int len = pkt->rx_ctrl.sig_len;

    // Minimum frame size check
    if (len < 24) return;

    // Check if it's an action frame (type=0, subtype=13)
    uint8_t frame_type = (payload[0] >> 2) & 0x03;
    uint8_t frame_subtype = (payload[0] >> 4) & 0x0F;

    if (frame_type != 0 || frame_subtype != 13) return;  // Not an action frame

    // Action frame body starts at offset 24
    if (len < 28) return;  // Need at least category + OUI

    // Check for vendor-specific action (category 127)
    if (payload[24] != 127) return;

    // Check for Espressif OUI
    if (memcmp(&payload[25], ESPRESSIF_OUI, 3) != 0) return;

    // This is an ESP-NOW packet! Extract source address (offset 10)
    const uint8_t *src_addr = &payload[10];

    // Find or add device
    int idx = find_espnow_device(src_addr);

    if (idx >= 0) {
        // Update existing entry
        espnow_scan_results[idx].rssi = pkt->rx_ctrl.rssi;
        espnow_scan_results[idx].last_seen = xTaskGetTickCount() * portTICK_PERIOD_MS;
        espnow_scan_results[idx].packet_count++;
    } else if (espnow_scan_result_count < MAX_ESPNOW_SCAN_RESULTS) {
        // Add new device
        idx = espnow_scan_result_count++;
        memcpy(espnow_scan_results[idx].addr, src_addr, 6);
        espnow_scan_results[idx].rssi = pkt->rx_ctrl.rssi;
        espnow_scan_results[idx].last_seen = xTaskGetTickCount() * portTICK_PERIOD_MS;
        espnow_scan_results[idx].packet_count = 1;
    }
}

// Initialize WiFi for ESP-NOW promiscuous scanning
static bool espnow_init_for_scan(void)
{
    esp_err_t ret;

    // Guard: prevent double initialization
    if (scanner_wifi_running || scanner_espnow_running || wifi_is_running) {
        ESP_LOGW(TAG, "WiFi already running (wifi=%d, scanner=%d, espnow=%d), shutting down first",
                 wifi_is_running, scanner_wifi_running, scanner_espnow_running);
        wifi_scanner_shutdown();
        espnow_scanner_shutdown();
        wifi_shutdown();
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    ESP_LOGI(TAG, "ESP-NOW init starting, heap free: %lu", esp_get_free_heap_size());

    // Initialize WiFi driver
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW: WiFi init failed: %s", esp_err_to_name(ret));
        return false;
    }

    // Set to STA mode
    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW: Failed to set WiFi mode: %s", esp_err_to_name(ret));
        esp_wifi_deinit();  // Clean up on failure
        return false;
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW: Failed to start WiFi: %s", esp_err_to_name(ret));
        esp_wifi_deinit();  // Clean up on failure
        return false;
    }

    // Enable promiscuous mode
    ret = esp_wifi_set_promiscuous(true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW: Failed to enable promiscuous mode: %s", esp_err_to_name(ret));
        esp_wifi_stop();
        esp_wifi_deinit();  // Clean up on failure
        return false;
    }

    // Set promiscuous callback
    ret = esp_wifi_set_promiscuous_rx_cb(espnow_promiscuous_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW: Failed to set promiscuous callback: %s", esp_err_to_name(ret));
        esp_wifi_set_promiscuous(false);
        esp_wifi_stop();
        esp_wifi_deinit();  // Clean up on failure
        return false;
    }

    // Filter for management frames only (more efficient)
    wifi_promiscuous_filter_t filter = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT
    };
    esp_wifi_set_promiscuous_filter(&filter);

    scanner_espnow_running = true;
    ESP_LOGI(TAG, "ESP-NOW promiscuous scanning initialized");
    return true;
}

// Shutdown ESP-NOW scanner
static void espnow_scanner_shutdown(void)
{
    if (scanner_espnow_running) {
        ESP_LOGI(TAG, "Shutting down ESP-NOW scanner...");

        espnow_scan_active = false;

        // Disable promiscuous mode
        esp_wifi_set_promiscuous(false);

        // Stop and deinit WiFi
        esp_wifi_stop();
        vTaskDelay(pdMS_TO_TICKS(100));  // Let events drain
        esp_wifi_deinit();

        // Allow hardware to fully release resources
        vTaskDelay(pdMS_TO_TICKS(100));

        scanner_espnow_running = false;
        ESP_LOGI(TAG, "ESP-NOW scanner shut down");
    }
}

// Task to scan for ESP-NOW (WiFi must already be initialized before calling)
static void espnow_scanner_task(void *pvParameters)
{
    ESP_LOGI(TAG, "ESP-NOW scanner task started");

    // Check if screen still active
    if (!scanner_screen_active) {
        scanner_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    // Clear previous results
    espnow_scan_result_count = 0;
    espnow_scan_active = false;

    // Give WiFi time to stabilize
    vTaskDelay(pdMS_TO_TICKS(500));

    // Update status and start scanning
    bsp_display_lock(0);
    if (scanner_screen_active && scanner_espnow_status) {
        lv_label_set_text(scanner_espnow_status, "Listening for ESP-NOW...");
    }
    bsp_display_unlock();

    // Enable scanning
    espnow_scan_active = true;

    // Scan across all channels (cycle through them)
    for (int cycle = 0; cycle < 2 && scanner_screen_active; cycle++) {  // 2 full cycles
        for (int channel = 1; channel <= 13 && scanner_screen_active; channel++) {
            if (!espnow_scan_active) break;

            esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

            // Update status with channel
            bsp_display_lock(0);
            if (scanner_screen_active && scanner_espnow_status) {
                char status[32];
                snprintf(status, sizeof(status), "Scanning ch %d... (%d found)",
                        channel, espnow_scan_result_count);
                lv_label_set_text(scanner_espnow_status, status);
            }
            bsp_display_unlock();

            // Listen on this channel for 300ms
            vTaskDelay(pdMS_TO_TICKS(300));
        }
    }

    espnow_scan_active = false;

    // Check if screen was destroyed during scan
    if (!scanner_screen_active) {
        scanner_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Found %d ESP-NOW devices", espnow_scan_result_count);

    // Sort by packet count (most active first)
    if (espnow_scan_result_count > 1) {
        qsort(espnow_scan_results, espnow_scan_result_count,
              sizeof(espnow_scan_result_t), compare_espnow_packets);
    }

    // Update UI with results
    bsp_display_lock(0);

    if (scanner_screen_active && scanner_espnow_status) {
        char status_text[32];
        snprintf(status_text, sizeof(status_text), "Found %d devices", espnow_scan_result_count);
        lv_label_set_text(scanner_espnow_status, status_text);
    }

    // Populate the list
    if (scanner_screen_active && scanner_espnow_list) {
        lv_obj_clean(scanner_espnow_list);

        if (espnow_scan_result_count == 0) {
            // No devices found message
            lv_obj_t *no_devices = lv_label_create(scanner_espnow_list);
            lv_label_set_text(no_devices, "No ESP-NOW devices detected\n\nDevices must be actively\ntransmitting to be seen");
            lv_obj_set_style_text_color(no_devices, lv_color_hex(0x3a5a7c), 0);
            lv_obj_set_style_text_font(no_devices, &lv_font_montserrat_14, 0);
            lv_obj_set_style_text_align(no_devices, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_center(no_devices);
        } else {
            for (int i = 0; i < espnow_scan_result_count; i++) {
                lv_obj_t *item = lv_obj_create(scanner_espnow_list);
                lv_obj_set_size(item, lv_pct(100), 42);
                lv_obj_set_style_bg_color(item, lv_color_hex(0x1a3a5c), 0);
                lv_obj_set_style_border_width(item, 0, 0);
                lv_obj_set_style_radius(item, 6, 0);
                lv_obj_set_style_pad_all(item, 6, 0);
                lv_obj_remove_flag(item, LV_OBJ_FLAG_SCROLLABLE);

                // MAC address
                lv_obj_t *addr_label = lv_label_create(item);
                char addr_str[18];
                snprintf(addr_str, sizeof(addr_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                        espnow_scan_results[i].addr[0], espnow_scan_results[i].addr[1],
                        espnow_scan_results[i].addr[2], espnow_scan_results[i].addr[3],
                        espnow_scan_results[i].addr[4], espnow_scan_results[i].addr[5]);
                lv_label_set_text(addr_label, addr_str);
                lv_obj_set_style_text_color(addr_label, lv_color_white(), 0);
                lv_obj_set_style_text_font(addr_label, &lv_font_montserrat_12, 0);
                lv_obj_align(addr_label, LV_ALIGN_TOP_LEFT, 0, 0);

                // Signal + packet count
                lv_obj_t *info_label = lv_label_create(item);
                char info_text[48];
                snprintf(info_text, sizeof(info_text), "%s %ddBm  %d pkts",
                        get_espnow_signal_bars(espnow_scan_results[i].rssi),
                        espnow_scan_results[i].rssi,
                        espnow_scan_results[i].packet_count);
                lv_label_set_text(info_label, info_text);
                lv_obj_set_style_text_color(info_label, lv_color_hex(0x88aacc), 0);
                lv_obj_set_style_text_font(info_label, &lv_font_montserrat_10, 0);
                lv_obj_align(info_label, LV_ALIGN_BOTTOM_LEFT, 0, 0);
            }
        }
    }

    bsp_display_unlock();

    ESP_LOGI(TAG, "ESP-NOW scan complete, task exiting");
    scanner_task_handle = NULL;
    vTaskDelete(NULL);
}

// ========================================================================
// END ESP-NOW SCANNER FUNCTIONS
// ========================================================================

// ========================================================================
// END CONFIGURE INITIAL TIME FROM NTP SERVER
// ========================================================================

// ========================================================================
// Start and manage RTC 
// ========================================================================

// Initialize RTC
static esp_err_t init_rtc(void)
{
    esp_err_t ret = bsp_i2c_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2C: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_master_bus_handle_t i2c_handle = bsp_i2c_get_handle();
    ret = pcf85063a_init(&rtc_dev, i2c_handle, PCF85063A_ADDRESS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init RTC: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "RTC initialized");
    return ESP_OK;
}

// Read RTC and check validity
static bool read_rtc_time(void)
{
    if (pcf85063a_get_time_date(&rtc_dev, &current_time) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read RTC");
        return false;
    }

    // Sanity check - year should be reasonable
    if (current_time.year < 2024 || current_time.year > 2100) {
        ESP_LOGW(TAG, "RTC time invalid (year: %d)", current_time.year);
        // Set default time
        current_time.year = 1970;
        current_time.month = 1;
        current_time.day = 1;
        current_time.dotw = 4; // Thursday
        current_time.hour = 0;
        current_time.min = 0;
        current_time.sec = 0;
        return false;
    }

    ESP_LOGI(TAG, "RTC time: %04d-%02d-%02d %02d:%02d:%02d",
             current_time.year, current_time.month, current_time.day,
             current_time.hour, current_time.min, current_time.sec);
    return true;
}

// Day of week names
static const char *day_names[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};

// Update time display on watchface (handles both watchface types)
static void update_time_display(void)
{
    // Skip if UI not ready or in sleep stage 2+ (UI destroyed)
    if (!watchface || on_settings_screen || current_sleep_stage >= 2) return;

    bsp_display_lock(0);

    // Re-check after acquiring lock (UI could have been destroyed while waiting)
    if (!watchface || current_sleep_stage >= 2) {
        bsp_display_unlock();
        return;
    }

    // Update time based on watchface type
    const char *day_name = (current_time.dotw <= 6) ? day_names[current_time.dotw] : "---";

    switch (current_watchface_type) {
        case WATCHFACE_TYPE_PILOT:
            pilot_watchface_set_time((pilot_watchface_t *)watchface,
                                     current_time.hour, current_time.min, current_time.sec);
            pilot_watchface_set_date((pilot_watchface_t *)watchface, day_name, current_time.day);
            break;
        case WATCHFACE_TYPE_SIMPLE:
        default:
            simple_watchface_set_time((simple_watchface_t *)watchface,
                                      current_time.hour, current_time.min, current_time.sec);
            simple_watchface_set_date((simple_watchface_t *)watchface, day_name, current_time.day);
            break;
    }

    bsp_display_unlock();
}

// ========================================================================
// End RTC 
// ========================================================================

// ========================================================================
// Start LVGL BATTERY WIDGET CREATION - CHARGE STATE AND CHARGHING INDICATION
// ========================================================================

// Create graphical battery widget (sideways AA battery style)
static void create_battery_widget(void)
{
    bsp_display_lock(0);

    // USB icon (only visible when VBUS connected)
    usb_icon = lv_label_create(lv_scr_act());
    lv_label_set_text(usb_icon, LV_SYMBOL_USB);
    lv_obj_set_style_text_color(usb_icon, lv_color_hex(0x00AAFF), 0);
    lv_obj_set_style_text_font(usb_icon, &lv_font_montserrat_20, 0);
    lv_obj_align(usb_icon, LV_ALIGN_TOP_RIGHT, -155, 10);  // Moved 30px left
    lv_obj_add_flag(usb_icon, LV_OBJ_FLAG_HIDDEN);  // Hidden by default
    lv_obj_move_foreground(usb_icon);

    // Charging icon (only visible when charging)
    charging_icon = lv_label_create(lv_scr_act());
    lv_label_set_text(charging_icon, LV_SYMBOL_CHARGE);
    lv_obj_set_style_text_color(charging_icon, lv_color_hex(0xFFD700), 0);  // Gold
    lv_obj_set_style_text_font(charging_icon, &lv_font_montserrat_20, 0);
    lv_obj_align(charging_icon, LV_ALIGN_TOP_RIGHT, -130, 10);  // Moved 30px left
    lv_obj_add_flag(charging_icon, LV_OBJ_FLAG_HIDDEN);  // Hidden by default
    lv_obj_move_foreground(charging_icon);

    // Container for the entire battery widget (always on top)
    battery_container = lv_obj_create(lv_scr_act());
    lv_obj_set_size(battery_container, 80, 40);
    lv_obj_align(battery_container, LV_ALIGN_TOP_RIGHT, -40, 5);  // Moved 30px left
    lv_obj_set_style_bg_opa(battery_container, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(battery_container, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(battery_container, 0, LV_PART_MAIN);
    lv_obj_move_foreground(battery_container);  // Keep on top

    // Battery body (main rectangle)
    battery_body = lv_obj_create(battery_container);
    lv_obj_set_size(battery_body, 60, 30);
    lv_obj_align(battery_body, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_color(battery_body, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_border_color(battery_body, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_border_width(battery_body, 2, LV_PART_MAIN);
    lv_obj_set_style_radius(battery_body, 3, LV_PART_MAIN);
    lv_obj_set_style_pad_all(battery_body, 2, LV_PART_MAIN);

    // Battery tip/terminal (small rectangle on right side)
    battery_tip = lv_obj_create(battery_container);
    lv_obj_set_size(battery_tip, 6, 16);
    lv_obj_align(battery_tip, LV_ALIGN_LEFT_MID, 60, 0);
    lv_obj_set_style_bg_color(battery_tip, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_border_width(battery_tip, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(battery_tip, 2, LV_PART_MAIN);

    // Battery fill level (bar inside body)
    battery_fill = lv_obj_create(battery_body);
    lv_obj_set_size(battery_fill, 50, 22);  // Will be adjusted based on percentage
    lv_obj_align(battery_fill, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_color(battery_fill, lv_color_hex(0x00FF00), LV_PART_MAIN);
    lv_obj_set_style_border_width(battery_fill, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(battery_fill, 2, LV_PART_MAIN);

    // Percentage label (on top of battery)
    battery_percent_label = lv_label_create(battery_container);
    lv_label_set_text(battery_percent_label, "100%");
    lv_obj_set_style_text_color(battery_percent_label, lv_color_hex(0xAA00FF), 0);  // Purple
    lv_obj_set_style_text_font(battery_percent_label, &lv_font_montserrat_12, 0);
    lv_obj_align(battery_percent_label, LV_ALIGN_CENTER, -5, 0);

    bsp_display_unlock();
}

// Update battery widget with current battery info
static void update_battery_display(void)
{
    // Skip if UI not ready or in sleep stage 2+ (UI destroyed)
    if (!battery_fill || !battery_percent_label || current_sleep_stage >= 2) return;

    battery_info_t info = get_battery_info();

    bsp_display_lock(0);

    // Re-check after acquiring lock (UI could have been destroyed while waiting)
    if (!battery_fill || !battery_percent_label || current_sleep_stage >= 2) {
        bsp_display_unlock();
        return;
    }

    // Show/hide USB icon based on VBUS connection
    if (usb_icon) {
        if (info.is_vbus_connected) {
            lv_obj_clear_flag(usb_icon, LV_OBJ_FLAG_HIDDEN);  // Show
            lv_obj_move_foreground(usb_icon);
        } else {
            lv_obj_add_flag(usb_icon, LV_OBJ_FLAG_HIDDEN);    // Hide
        }
    }

    // Show/hide charging icon based on charging status
    if (charging_icon) {
        if (info.is_charging) {
            lv_obj_clear_flag(charging_icon, LV_OBJ_FLAG_HIDDEN);  // Show
            lv_obj_move_foreground(charging_icon);
        } else {
            lv_obj_add_flag(charging_icon, LV_OBJ_FLAG_HIDDEN);    // Hide
        }
    }

    // Calculate fill width based on percentage (max 50px for 100%)
    int fill_width = (info.battery_percent * 50) / 100;
    if (fill_width < 2) fill_width = 2;  // Minimum visible fill
    lv_obj_set_width(battery_fill, fill_width);

    // Color code based on battery level (always green when charging, otherwise by level)
    uint32_t fill_color;
    if (info.is_charging) {
        // Charging: Keep green to show healthy charge
        fill_color = 0x00FF00;  // Green
    } else if (info.battery_percent > 25) {
        fill_color = 0x00FF00;  // Green (>25%)
    } else if (info.battery_percent > 10) {
        fill_color = 0xFFAA00;  // Orange (11-25%)
    } else {
        fill_color = 0xFF0000;  // Red (<=10%)
    }
    lv_obj_set_style_bg_color(battery_fill, lv_color_hex(fill_color), LV_PART_MAIN);

    // Update percentage text
    char percent_str[8];
    snprintf(percent_str, sizeof(percent_str), "%d%%", info.battery_percent);
    lv_label_set_text(battery_percent_label, percent_str);

    // Move battery container forward to ensure it's visible on top
    lv_obj_move_foreground(battery_container);

    bsp_display_unlock();
}


// ========================================================================
// END LVGL BATTERY WIDGET CREATION - CHARGE STATE AND CHARGHING INDICATION
// ========================================================================


// ========================================================================
// Time display task - updates clock and battery every second
// ========================================================================

// Pauses updates during sleep stages 2+ (UI destroyed, no display to update)
static void time_display_task(void *pvParameters)
{
    int battery_update_counter = 0;

    while (1) {
        // Stage 3: Display powered off - sleep for 5 seconds to minimize CPU usage
        if (current_sleep_stage >= 3) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        // Stage 2: Display off but powered - sleep for 2 seconds
        if (current_sleep_stage >= 2) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        // Read current RTC time (no logging - too noisy!)
        pcf85063a_get_time_date(&rtc_dev, &current_time);

        // Update time display
        update_time_display();

        // Update battery display every 5 seconds (not every second to reduce I2C traffic)
        battery_update_counter++;
        if (battery_update_counter >= 5) {
            update_battery_display();
            battery_update_counter = 0;
        }

        // Wait 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


// ========================================================================
// End time display task
// ========================================================================

// ========================================================================
// START IMU TASK - FOR FUTURE GESTURE SUPPORT
// ========================================================================

// During sleep stages 2+, only performs wake detection at reduced rate
static void imu_display_task(void *pvParameters)
{
    qmi8658_data_t data;

    // Twist gesture state tracking (Z-axis only for simple wrist rotation)
    static float last_gyro_z = 0;
    static uint32_t twist_start_time = 0;
    static bool twist_phase_1 = false;  // Detected first rotation
    static bool phase_1_positive = false;  // Direction of first twist

    ESP_LOGI(TAG, "IMU task started with twist-to-wake gesture");

    while (1) {
        if (qmi8658_read_sensor_data(&imu_dev, &data) == ESP_OK) {
            // Update watchface sub-dials with accelerometer data
            // Skip during sleep stages 2+ (UI is destroyed), settings screen,
            // app screens (scanner/audio), or when paused during transitions
            if (watchface && current_sleep_stage < 2 && !on_settings_screen &&
                !imu_updates_paused && current_screen == SCREEN_WATCHFACE) {
                bsp_display_lock(0);
                // Re-check watchface validity after acquiring lock (could have been deleted)
                if (!watchface || imu_updates_paused || current_screen != SCREEN_WATCHFACE) {
                    bsp_display_unlock();
                } else {
                    switch (current_watchface_type) {
                        case WATCHFACE_TYPE_PILOT:
                            pilot_watchface_set_accel((pilot_watchface_t *)watchface,
                                                      data.accelX, data.accelY, data.accelZ);
                            break;
                        case WATCHFACE_TYPE_SIMPLE:
                        default:
                            // Simple watchface doesn't display accel (stub function)
                            simple_watchface_set_accel((simple_watchface_t *)watchface,
                                                       data.accelX, data.accelY, data.accelZ);
                            break;
                    }
                    bsp_display_unlock();
                }
            }

            // Twist-to-wake: Detect wrist rotation using Z-axis only
            // Twist one direction, then reverse - works regardless of starting orientation
            // Active when screen is off OR dimmed
            if (!backlight_on || display_dimmed) {
                uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

                // Calculate Z rotation delta from previous sample
                float delta_z = data.gyroZ - last_gyro_z;

                // Phase 1: Detect initial twist (significant Z rotation either direction)
                if (!twist_phase_1) {
                    if (delta_z > TWIST_THRESHOLD) {
                        twist_phase_1 = true;
                        phase_1_positive = true;
                        twist_start_time = now;
                        ESP_LOGI(TAG, "Twist phase 1 detected (dZ:%.1f positive)", delta_z);
                    } else if (delta_z < -TWIST_THRESHOLD) {
                        twist_phase_1 = true;
                        phase_1_positive = false;
                        twist_start_time = now;
                        ESP_LOGI(TAG, "Twist phase 1 detected (dZ:%.1f negative)", delta_z);
                    }
                }
                // Phase 2: Detect counter-rotation (opposite direction from phase 1)
                else {
                    bool reverse_detected = (phase_1_positive && delta_z < -TWIST_THRESHOLD) ||
                                           (!phase_1_positive && delta_z > TWIST_THRESHOLD);

                    if (reverse_detected && (now - twist_start_time) < TWIST_TIMEOUT_MS) {
                        // SUCCESS - Complete twist gesture detected!
                        // Just reset counter - let sleep_management_task handle proper wake sequence
                        inactivity_counter = 0;
                        ESP_LOGI(TAG, "TWIST WAKE - Gesture completed (%.1fms)",
                                 (float)(now - twist_start_time));
                        twist_phase_1 = false;  // Reset for next gesture
                    }
                    // Timeout - reset if too much time passed
                    else if ((now - twist_start_time) > TWIST_TIMEOUT_MS) {
                        twist_phase_1 = false;
                        ESP_LOGI(TAG, "Twist timeout - reset");
                    }
                }

                // Store current Z value for next iteration
                last_gyro_z = data.gyroZ;
            }
            // Reset twist detection when screen is fully on (not dimmed)
            else {
                twist_phase_1 = false;
                last_gyro_z = data.gyroZ;
            }
        } else {
            ESP_LOGW(TAG, "Failed to read IMU data");
        }

        // Adjust polling rate based on sleep stage
        if (current_sleep_stage >= 3) {
            vTaskDelay(pdMS_TO_TICKS(500));  // 2Hz in Stage 3 - minimal for twist detection
        } else if (current_sleep_stage >= 2) {
            vTaskDelay(pdMS_TO_TICKS(250));  // 4Hz in Stage 2
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));  // 10Hz for display updates
        }
    }
}


// ========================================================================
// END IMU TASK - FOR FUTURE GESTURE SUPPORT
// ========================================================================

// ========================================================================
// START GLOBAL SLEEP MANAGEMENT TASK
// ========================================================================

// Sleep management task - sequential power saving stages
// Stage 1: Dim to 10% after 20s
// Stage 2: Backlight off after 30s
// Stage 3: Display power off (ALDO2) + Audio power off (ALDO1) after 40s
// Touch or IMU twist resets timer and wakes from any stage
static void sleep_management_task(void *pvParameters)
{
    uint8_t last_stage = 0;

    ESP_LOGI(TAG, "Sleep management started - waiting for WiFi complete...");

    // Wait for WiFi to complete before starting timer
    while (!wifi_init_complete) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "WiFi complete - sleep management active");
    ESP_LOGI(TAG, "  Stage 1: Dim at %ds, Stage 2: Off at %ds, Stage 3: Power off at %ds",
             STAGE_DIM_THRESHOLD, STAGE_OFF_THRESHOLD, STAGE_SLEEP_THRESHOLD);

    while (1) {
        inactivity_counter++;

        // Debug: log counter every 2 minutes in Stage 3 (reduced frequency to save power)
        if (inactivity_counter % 120 == 0 && inactivity_counter > 0) {
            ESP_LOGI(TAG, "Sleep timer: %u seconds (stage %d)", (unsigned int)inactivity_counter, current_sleep_stage);
        }

        // Check for touch detection (from esp_lcd_touch interrupt callback) and log it
        if (touch_detected_flag) {
            touch_detected_flag = false;
            ESP_LOGI(TAG, ">>> TOUCH DETECTED (esp_lcd_touch interrupt) <<<");
        }

        // Stage 1: Dim to 10% after 20s
        if (inactivity_counter >= STAGE_DIM_THRESHOLD && last_stage < 1) {
            ESP_LOGI(TAG, "Stage 1: Dimming backlight to 10%% (%ds idle)", STAGE_DIM_THRESHOLD);
            bsp_display_brightness_set(10);
            display_dimmed = true;
            last_stage = 1;
            current_sleep_stage = 1;
        }

        // Stage 2: Turn off backlight after 30s, destroy UI to save CPU
        if (inactivity_counter >= STAGE_OFF_THRESHOLD && last_stage < 2) {
            ESP_LOGI(TAG, "Stage 2: Backlight OFF + UI destroyed (%ds idle)", STAGE_OFF_THRESHOLD);
            bsp_display_backlight_off();
            backlight_on = false;
            display_dimmed = false;  // No longer dimmed, now off
            destroy_ui();  // Free UI resources, save CPU cycles
            last_stage = 2;
            current_sleep_stage = 2;
        }

        // Stage 3: Power off display (ALDO2) after 40s for maximum power savings
        if (inactivity_counter >= STAGE_SLEEP_THRESHOLD && last_stage < 3) {
            ESP_LOGI(TAG, "Stage 3: Display power OFF (%ds idle)", STAGE_SLEEP_THRESHOLD);
            pmu_display_power_off();  // ALDO2 - cuts display power completely
            last_stage = 3;
            current_sleep_stage = 3;
        }

        // Reset stages if user became active (wrist twist or touch reset inactivity_counter)
        if (inactivity_counter < STAGE_DIM_THRESHOLD && last_stage > 0) {
            ESP_LOGI(TAG, "User active - waking from stage %d", last_stage);

            // Stage 3: Need to restore display power and reinit controller
            if (last_stage == 3) {
                ESP_LOGI(TAG, "Restoring display power (ALDO2)...");
                pmu_display_power_on();
                vTaskDelay(pdMS_TO_TICKS(200));  // Wait for power to stabilize

                // Reinitialize display controller
                esp_err_t ret = display_reinit_from_lvgl();
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Display reinit failed: %s", esp_err_to_name(ret));
                }
            }

            // Stage 2+: UI was destroyed, need to recreate it
            // BUT only if we're still on the watchface - user may have already
            // navigated to another screen (scanner/audio) during wake
            if (last_stage >= 2 && current_screen == SCREEN_WATCHFACE) {
                ESP_LOGI(TAG, "Recreating watchface UI...");
                on_settings_screen = false;
                create_ui();

                // Immediately update with current time from RTC
                pcf85063a_get_time_date(&rtc_dev, &current_time);
                update_time_display();
                update_battery_display();

                // Wait for display to render before turning on backlight
                vTaskDelay(pdMS_TO_TICKS(800));
            } else if (last_stage >= 2) {
                ESP_LOGI(TAG, "Skipping UI recreate - already on screen %d", current_screen);
                // Still need delay for display to stabilize
                vTaskDelay(pdMS_TO_TICKS(200));
            }

            // Turn on backlight
            bsp_display_backlight_on();
            backlight_on = true;
            display_dimmed = false;
            last_stage = 0;
            current_sleep_stage = 0;  // Signal tasks to resume normal operation
            ESP_LOGI(TAG, "Display restored to full brightness");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));  // Check every second
    }
}


// ========================================================================
// END GLOBAL SLEEP MANAGEMENT TASK
// ========================================================================

// ========================================================================
// START TOUCH HANDLING (esp_lcd_touch interrupt callback)
// ========================================================================

// Store the LVGL indev for use in interrupt callback
static lv_indev_t *g_touch_indev = NULL;

// Touch interrupt callback - called by esp_lcd_touch when GPIO interrupt fires
// This REPLACES the default lvgl_port callback, so we must call lvgl_port_task_wake ourselves
static void IRAM_ATTR touch_interrupt_callback(esp_lcd_touch_handle_t tp)
{
    // Set our wake flags
    touch_detected_flag = true;
    inactivity_counter = 0;

    // Wake LVGL task so it reads the touch data (this is what lvgl_port's callback does)
    if (g_touch_indev != NULL) {
        lvgl_port_task_wake(LVGL_PORT_EVENT_TOUCH, g_touch_indev);
    }
}

// Initialize touch wake detection by registering our interrupt callback
// The BSP already initialized the touch hardware via bsp_display_start() -> bsp_touch_new()
// We register our callback to intercept touch interrupts for wake detection
static void init_touch_wake_interrupt(void)
{
    // Get the LVGL touch input device created by BSP
    lv_indev_t *touch_indev = bsp_display_get_input_dev();
    if (touch_indev == NULL) {
        ESP_LOGE(TAG, "Touch input device not available - BSP may not have initialized touch");
        return;
    }

    // Store for use in interrupt callback
    g_touch_indev = touch_indev;

    // Get the esp_lcd_touch_handle_t from the LVGL indev driver data
    // The driver data is lvgl_port_touch_ctx_t* where first member is the handle
    void *driver_data = lv_indev_get_driver_data(touch_indev);
    if (driver_data == NULL) {
        ESP_LOGE(TAG, "Touch driver data not available");
        return;
    }

    // First member of lvgl_port_touch_ctx_t is esp_lcd_touch_handle_t
    esp_lcd_touch_handle_t tp = *((esp_lcd_touch_handle_t *)driver_data);
    if (tp == NULL) {
        ESP_LOGE(TAG, "Touch handle not available");
        return;
    }

    // Register our interrupt callback (replaces lvgl_port's default callback)
    esp_err_t ret = esp_lcd_touch_register_interrupt_callback(tp, touch_interrupt_callback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register touch interrupt callback: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Touch interrupt callback registered for wake detection");
}

// ========================================================================
// END TOUCH HANDLING
// ========================================================================

// ========================================================================
// START WATCHFACE SELECTION UI
// ========================================================================

// Forward declarations
static void create_ui(void);
static void destroy_ui(void);

// Settings screen UI components
static lv_obj_t *settings_roller = NULL;
static lv_obj_t *settings_return_btn = NULL;

// Return to watchface from settings (called by return button or after selection)
static void return_to_watchface(void)
{
    ESP_LOGI(TAG, "Returning to watchface (type %d)", current_watchface_type);

    bsp_display_lock(0);

    // Clean the settings screen
    lv_obj_t *scr = lv_scr_act();
    if (scr) {
        lv_obj_clean(scr);
    }
    settings_roller = NULL;
    settings_return_btn = NULL;

    bsp_display_unlock();

    // Mark that we're leaving settings
    on_settings_screen = false;

    // Create the watchface UI
    create_ui();

    // Reset inactivity counter
    inactivity_counter = 0;
}

// Roller value changed callback
static void roller_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *roller = (lv_obj_t *)lv_event_get_target(e);
        uint32_t selected = lv_roller_get_selected(roller);

        ESP_LOGI(TAG, "Watchface selected: %lu", selected);

        // Only save and switch if different from current
        if ((int)selected != current_watchface_type) {
            current_watchface_type = (int)selected;
            save_watchface_type_to_nvs(current_watchface_type);

            // Return to watchface with new selection
            return_to_watchface();
        }
    }
}

// Return button callback
static void return_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_CLICKED) {
        ESP_LOGI(TAG, "Return button pressed");
        return_to_watchface();
    }
}

// Show watchface selector screen
static void show_watchface_selector(void)
{
    ESP_LOGI(TAG, "Opening watchface selector");

    // Mark that we're on settings screen
    on_settings_screen = true;

    // Destroy current UI (similar to stage 2 sleep)
    destroy_ui();

    bsp_display_lock(0);

    lv_obj_t *scr = lv_scr_act();

    // Set black background
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    // Add global event handler for touch (resets inactivity)
    lv_obj_add_event_cb(scr, [](lv_event_t *e) {
        lv_event_code_t code = lv_event_get_code(e);
        if (code == LV_EVENT_PRESSED || code == LV_EVENT_PRESSING ||
            code == LV_EVENT_CLICKED || code == LV_EVENT_RELEASED) {
            inactivity_counter = 0;
        }
    }, LV_EVENT_ALL, NULL);

    // Title label
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "Select Watchface");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 60);

    // Create roller for watchface selection (infinite/circular scrolling)
    settings_roller = lv_roller_create(scr);
    lv_roller_set_options(settings_roller, watchface_names, LV_ROLLER_MODE_INFINITE);
    lv_roller_set_visible_row_count(settings_roller, 3);
    lv_roller_set_selected(settings_roller, current_watchface_type, LV_ANIM_OFF);

    // Style the roller
    lv_obj_set_width(settings_roller, 280);
    lv_obj_set_style_text_font(settings_roller, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(settings_roller, lv_color_white(), 0);
    lv_obj_set_style_text_color(settings_roller, lv_color_make(0x00, 0xFF, 0x00), LV_PART_SELECTED);
    lv_obj_set_style_bg_color(settings_roller, lv_color_make(0x30, 0x30, 0x30), 0);
    lv_obj_set_style_bg_color(settings_roller, lv_color_make(0x50, 0x50, 0x50), LV_PART_SELECTED);
    lv_obj_set_style_bg_opa(settings_roller, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_opa(settings_roller, LV_OPA_COVER, LV_PART_SELECTED);
    lv_obj_set_style_border_width(settings_roller, 2, 0);
    lv_obj_set_style_border_color(settings_roller, lv_color_make(0x80, 0x80, 0x80), 0);
    lv_obj_set_style_radius(settings_roller, 10, 0);
    lv_obj_align(settings_roller, LV_ALIGN_CENTER, 0, 0);

    // Add event callback
    lv_obj_add_event_cb(settings_roller, roller_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Create return button (lower right)
    settings_return_btn = lv_btn_create(scr);
    lv_obj_set_size(settings_return_btn, 100, 50);
    lv_obj_align(settings_return_btn, LV_ALIGN_BOTTOM_RIGHT, -20, -40);
    lv_obj_set_style_bg_color(settings_return_btn, lv_color_make(0x40, 0x40, 0x40), 0);
    lv_obj_set_style_bg_color(settings_return_btn, lv_color_make(0x60, 0x60, 0x60), LV_STATE_PRESSED);
    lv_obj_set_style_radius(settings_return_btn, 10, 0);

    // Button label
    lv_obj_t *btn_label = lv_label_create(settings_return_btn);
    lv_label_set_text(btn_label, "Return");
    lv_obj_set_style_text_color(btn_label, lv_color_white(), 0);
    lv_obj_center(btn_label);

    // Add event callback
    lv_obj_add_event_cb(settings_return_btn, return_btn_event_cb, LV_EVENT_CLICKED, NULL);

    bsp_display_unlock();

    // Reset inactivity counter
    inactivity_counter = 0;

    ESP_LOGI(TAG, "Watchface selector displayed");
}

// ========================================================================
// END WATCHFACE SELECTION UI
// ========================================================================

// ========================================================================
// START MULTI-SCREEN NAVIGATION
// ========================================================================

// Destroy only the watchface, keep battery widget
static void destroy_watchface_only(void)
{
    bsp_display_lock(0);

    if (watchface) {
        switch (current_watchface_type) {
            case WATCHFACE_TYPE_PILOT:
                pilot_watchface_destroy((pilot_watchface_t *)watchface);
                break;
            case WATCHFACE_TYPE_SIMPLE:
            default:
                simple_watchface_destroy((simple_watchface_t *)watchface);
                break;
        }
        watchface = NULL;
        ESP_LOGI(TAG, "Watchface destroyed (battery kept)");
    }

    bsp_display_unlock();

    // Allow LVGL to complete cleanup before continuing
    vTaskDelay(pdMS_TO_TICKS(100));
}

// Ensure battery widget stays on top of other UI elements
static void move_battery_to_foreground(void)
{
    bsp_display_lock(0);

    if (usb_icon) lv_obj_move_foreground(usb_icon);
    if (charging_icon) lv_obj_move_foreground(charging_icon);
    if (battery_container) lv_obj_move_foreground(battery_container);

    bsp_display_unlock();
}

// Scanner return button callback
static void scanner_return_btn_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        nav_return_to_watchface();
    }
}

// Audio return button callback
static void audio_return_btn_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        nav_return_to_watchface();
    }
}

// Apps return button callback
static void apps_return_btn_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        nav_return_to_watchface();
    }
}

// Forward declaration for create_apps_screen (needed by time_settings_exit_cb)
static void create_apps_screen(void);

// Time settings exit callback - returns to apps screen
static void time_settings_exit_cb(void)
{
    ESP_LOGI(TAG, "Exiting time settings, returning to apps screen");

    bsp_display_lock(0);

    // Destroy time settings screen
    if (time_settings_screen) {
        time_settings_destroy(time_settings_screen);
        time_settings_screen = NULL;
    }

    bsp_display_unlock();

    on_time_settings_screen = false;

    // Allow LVGL to complete cleanup
    vTaskDelay(pdMS_TO_TICKS(100));

    // Recreate apps screen
    create_apps_screen();
    move_battery_to_foreground();
    inactivity_counter = 0;

    ESP_LOGI(TAG, "Back on Apps screen");
}

// Time settings mode change callback
static void time_settings_mode_cb(bool is_manual)
{
    ESP_LOGI(TAG, "Time mode changed to: %s", is_manual ? "Manual" : "Auto");

    // Update global mode and save to NVS
    time_mode_is_manual = is_manual;
    save_time_mode_to_nvs(is_manual);

    // When switching to manual mode, set the roller values to current time
    if (is_manual && time_settings_screen) {
        pcf85063a_get_time_date(&rtc_dev, &current_time);
        time_settings_set_rollers(time_settings_screen,
                                   current_time.hour,
                                   current_time.min,
                                   current_time.month,
                                   current_time.day,
                                   current_time.year);
        ESP_LOGI(TAG, "Rollers set to current time: %02d:%02d %02d/%02d/%04d",
                 current_time.hour, current_time.min,
                 current_time.month, current_time.day, current_time.year);
    }
}

// Time settings set callback - sets RTC from manual input
static void time_settings_set_cb(int hour, int minute, int month, int day, int year)
{
    ESP_LOGI(TAG, "Manual time set: %02d:%02d %02d/%02d/%04d", hour, minute, month, day, year);

    // Create RTC time structure
    pcf85063a_datetime_t new_time = {
        .year = (uint16_t)year,
        .month = (uint8_t)month,
        .day = (uint8_t)day,
        .dotw = 0,  // Day of week - will be calculated or can be left as 0
        .hour = (uint8_t)hour,
        .min = (uint8_t)minute,
        .sec = 0  // Reset seconds to 0 when manually setting time
    };

    // Set the RTC
    esp_err_t ret = pcf85063a_set_time_date(&rtc_dev, new_time);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "RTC updated successfully");
        // Update the time display immediately
        if (time_settings_screen) {
            time_settings_update_time(time_settings_screen, hour, minute, 0);
        }
        // Also update the global current_time
        pcf85063a_get_time_date(&rtc_dev, &current_time);
    } else {
        ESP_LOGE(TAG, "Failed to set RTC: %s", esp_err_to_name(ret));
    }
}

// "Hold please" screen for NTP refresh
static lv_obj_t *hold_please_container = NULL;
static lv_obj_t *hold_please_label = NULL;
static TaskHandle_t ntp_refresh_task_handle = NULL;

// Forward declaration for refresh callback (used by ntp_refresh_task)
static void time_settings_refresh_cb(void);

// Helper to update the hold please screen text
static void update_hold_please_text(const char *text, uint32_t color)
{
    if (hold_please_label) {
        bsp_display_lock(0);
        lv_label_set_text(hold_please_label, text);
        lv_obj_set_style_text_color(hold_please_label, lv_color_hex(color), 0);
        bsp_display_unlock();
    }
}

// NTP refresh task - runs WiFi/NTP in separate task so LVGL can render
static void ntp_refresh_task(void *pvParameters)
{
    ESP_LOGI(TAG, "NTP refresh task started");

    bool refresh_success = false;

    // Reset NTP sync flag
    ntp_synced = false;
    s_retry_num = 0;

    // Clear any previous WiFi failure bits
    if (s_wifi_event_group) {
        xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT | WIFI_CONNECTED_BIT);
    }

    // Declare variables before any goto
    int wait_count = 0;
    bool wifi_connected = false;

    // Initialize WiFi
    ESP_LOGI(TAG, "Starting WiFi for NTP refresh...");
    if (!wifi_init()) {
        ESP_LOGW(TAG, "WiFi init failed during refresh");
        update_hold_please_text("WiFi init failed", 0xff4444);
        inactivity_counter = 0;
        vTaskDelay(pdMS_TO_TICKS(2000));
        goto refresh_done;
    }

    update_hold_please_text("Waiting for WiFi...", 0xffffff);

    // Wait for NTP sync or timeout (max 30 seconds for refresh)
    while (!ntp_synced && wait_count < 30) {
        // Keep resetting inactivity counter to prevent sleep stages during refresh
        inactivity_counter = 0;

        EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);

        // Check for WiFi connection success
        if ((bits & WIFI_CONNECTED_BIT) && !wifi_connected) {
            wifi_connected = true;
            update_hold_please_text("Syncing time...", 0xffffff);
        }

        // Check for WiFi failure
        if (bits & WIFI_FAIL_BIT) {
            ESP_LOGW(TAG, "WiFi connection failed during refresh");
            update_hold_please_text("WiFi failed", 0xff4444);
            inactivity_counter = 0;
            vTaskDelay(pdMS_TO_TICKS(2000));
            wifi_shutdown();
            goto refresh_done;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
        wait_count++;
    }

    if (ntp_synced) {
        ESP_LOGI(TAG, "NTP refresh successful!");
        update_hold_please_text("Sync complete!", 0x44ff44);
        refresh_success = true;
        inactivity_counter = 0;
        vTaskDelay(pdMS_TO_TICKS(1500));
    } else {
        ESP_LOGW(TAG, "NTP refresh timeout after %d seconds", wait_count);
        update_hold_please_text("Sync timeout", 0xff4444);
        inactivity_counter = 0;
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    // Shutdown WiFi
    inactivity_counter = 0;
    wifi_shutdown();

refresh_done:
    ESP_LOGI(TAG, "NTP refresh complete (success=%d), recreating time settings", refresh_success);

    bsp_display_lock(0);

    // Destroy "hold please" screen
    if (hold_please_container) {
        lv_obj_del(hold_please_container);
        hold_please_container = NULL;
        hold_please_label = NULL;
    }

    // Recreate time settings screen
    lv_obj_t *scr = lv_scr_act();
    time_settings_screen = time_settings_create(scr, time_settings_exit_cb, time_settings_mode_cb, time_settings_refresh_cb, time_settings_set_cb);

    bsp_display_unlock();

    if (time_settings_screen) {
        // Update time display with current RTC time
        pcf85063a_get_time_date(&rtc_dev, &current_time);
        time_settings_update_time(time_settings_screen, current_time.hour, current_time.min, current_time.sec);
        // Set mode switch to saved state (should be auto since refresh only available in auto mode)
        time_settings_set_mode(time_settings_screen, time_mode_is_manual);
        move_battery_to_foreground();
        on_time_settings_screen = true;
    }

    // Resume IMU updates
    imu_updates_paused = false;
    inactivity_counter = 0;

    ESP_LOGI(TAG, "Refresh complete, back on time settings screen");

    // Clean up task handle and delete self
    ntp_refresh_task_handle = NULL;
    vTaskDelete(NULL);
}

// Time settings refresh callback - destroys screen, shows status, spawns WiFi task
static void time_settings_refresh_cb(void)
{
    ESP_LOGI(TAG, "Refresh button pressed - starting NTP sync");

    // Don't start another refresh if one is already running
    if (ntp_refresh_task_handle != NULL) {
        ESP_LOGW(TAG, "NTP refresh already in progress");
        return;
    }

    // Pause IMU updates during refresh
    imu_updates_paused = true;

    bsp_display_lock(0);

    // Destroy time settings screen
    if (time_settings_screen) {
        time_settings_destroy(time_settings_screen);
        time_settings_screen = NULL;
    }

    // Create "hold please" screen
    lv_obj_t *scr = lv_scr_act();
    hold_please_container = lv_obj_create(scr);
    lv_obj_set_size(hold_please_container, 410, 502);
    lv_obj_center(hold_please_container);
    lv_obj_set_style_bg_color(hold_please_container, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(hold_please_container, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(hold_please_container, 0, 0);
    lv_obj_set_style_radius(hold_please_container, 0, 0);
    lv_obj_remove_flag(hold_please_container, LV_OBJ_FLAG_SCROLLABLE);

    hold_please_label = lv_label_create(hold_please_container);
    lv_label_set_text(hold_please_label, "Connecting...");
    lv_obj_set_style_text_color(hold_please_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(hold_please_label, &lv_font_montserrat_24, 0);
    lv_obj_center(hold_please_label);

    bsp_display_unlock();

    // Reset inactivity to keep screen awake
    inactivity_counter = 0;

    // Spawn task to handle WiFi/NTP (so LVGL can render the status screen)
    xTaskCreate(ntp_refresh_task, "ntp_refresh", 4096, NULL, 5, &ntp_refresh_task_handle);
}

// Clock button callback - opens time settings app
static void apps_clock_btn_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        ESP_LOGI(TAG, "Clock button pressed - opening time settings");

        bsp_display_lock(0);

        // Destroy apps screen
        if (apps_container) {
            lv_obj_del(apps_container);
            apps_container = NULL;
            apps_return_btn = NULL;
            apps_clock_btn = NULL;
        }

        bsp_display_unlock();

        // Allow LVGL to complete cleanup
        vTaskDelay(pdMS_TO_TICKS(100));

        // Create time settings screen
        bsp_display_lock(0);
        lv_obj_t *scr = lv_scr_act();
        time_settings_screen = time_settings_create(scr, time_settings_exit_cb, time_settings_mode_cb, time_settings_refresh_cb, time_settings_set_cb);
        bsp_display_unlock();

        if (time_settings_screen) {
            on_time_settings_screen = true;
            // Update time display with current RTC time
            pcf85063a_get_time_date(&rtc_dev, &current_time);
            time_settings_update_time(time_settings_screen, current_time.hour, current_time.min, current_time.sec);
            // Set mode switch to saved state from NVS
            time_settings_set_mode(time_settings_screen, time_mode_is_manual);
            // If manual mode, also populate the rollers with current time
            if (time_mode_is_manual) {
                time_settings_set_rollers(time_settings_screen,
                                          current_time.hour,
                                          current_time.min,
                                          current_time.month,
                                          current_time.day,
                                          current_time.year);
            }
            move_battery_to_foreground();
            inactivity_counter = 0;
            ESP_LOGI(TAG, "Time settings screen created (mode: %s)", time_mode_is_manual ? "Manual" : "Auto");
        } else {
            ESP_LOGE(TAG, "Failed to create time settings screen");
            // Fallback: recreate apps screen
            create_apps_screen();
            move_battery_to_foreground();
        }
    }
}

// Helper to create a scrollable list inside a tab
static lv_obj_t* create_scanner_list(lv_obj_t *parent)
{
    lv_obj_t *list = lv_obj_create(parent);
    lv_obj_set_size(list, lv_pct(100), 280);
    lv_obj_align(list, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(list, lv_color_hex(0x0a1628), 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_style_radius(list, 0, 0);
    lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_flex_main_place(list, LV_FLEX_ALIGN_START, 0);
    lv_obj_set_style_pad_row(list, 6, 0);
    lv_obj_set_style_pad_all(list, 4, 0);
    return list;
}

// Tab change event handler
static void scanner_tab_changed_cb(lv_event_t *e)
{
    if (scanner_switching_tabs) return;  // Prevent recursion

    lv_obj_t *tabview = (lv_obj_t *)lv_event_get_target(e);
    uint32_t new_tab = lv_tabview_get_tab_active(tabview);

    if (new_tab == scanner_active_tab) return;

    ESP_LOGI(TAG, "Scanner tab changed from %d to %lu", scanner_active_tab, new_tab);
    scanner_switching_tabs = true;

    // Stop current scan task
    if (scanner_task_handle != NULL) {
        vTaskDelete(scanner_task_handle);
        scanner_task_handle = NULL;
    }

    // Shutdown current radio based on previous tab
    if (scanner_active_tab == 0 && scanner_wifi_running) {
        wifi_scanner_shutdown();
    }
#if CONFIG_BT_ENABLED
    if (scanner_active_tab == 1 && scanner_bt_running) {
        ble_scanner_shutdown();
    }
#endif
    if (scanner_active_tab == 2 && scanner_espnow_running) {
        espnow_scanner_shutdown();
    }

    scanner_active_tab = new_tab;

    // Delay for radio switch (BLE needs time to fully release memory)
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "After radio switch delay, heap free: %lu", esp_get_free_heap_size());

    if (new_tab == 0) {
        // WiFi tab - init WiFi BEFORE creating task to ensure heap is available
        bsp_display_lock(0);
        if (scanner_wifi_status) {
            lv_label_set_text(scanner_wifi_status, "Initializing WiFi...");
        }
        if (scanner_wifi_list) {
            lv_obj_clean(scanner_wifi_list);
        }
        bsp_display_unlock();

        // Init WiFi first (needs ~40KB heap, must happen before task stack allocation)
        if (!wifi_init_for_scan()) {
            bsp_display_lock(0);
            if (scanner_wifi_status) {
                lv_label_set_text(scanner_wifi_status, "WiFi init failed - low memory");
            }
            bsp_display_unlock();
            scanner_switching_tabs = false;
            return;
        }

        // Now create task with smaller stack (WiFi already initialized)
        xTaskCreate(wifi_scanner_task, "wifi_scan", 4096, NULL, 3, &scanner_task_handle);
    } else if (new_tab == 1) {
        // Bluetooth tab - init BLE BEFORE creating task
#if CONFIG_BT_ENABLED
        bsp_display_lock(0);
        if (scanner_bt_status) {
            lv_label_set_text(scanner_bt_status, "Initializing BLE...");
        }
        if (scanner_bt_list) {
            lv_obj_clean(scanner_bt_list);
        }
        bsp_display_unlock();

        // Init BLE first (must happen before task stack allocation)
        if (!ble_init_for_scan()) {
            bsp_display_lock(0);
            if (scanner_bt_status) {
                lv_label_set_text(scanner_bt_status, "BLE init failed - low memory");
            }
            bsp_display_unlock();
            scanner_switching_tabs = false;
            return;
        }

        // Now create task with smaller stack (BLE already initialized)
        xTaskCreate(bt_scanner_task, "bt_scan", 6144, NULL, 3, &scanner_task_handle);
#else
        bsp_display_lock(0);
        if (scanner_bt_status) {
            lv_label_set_text(scanner_bt_status, "BT not enabled in sdkconfig");
        }
        bsp_display_unlock();
#endif
    } else if (new_tab == 2) {
        // ESP-NOW tab - init WiFi BEFORE creating task
        bsp_display_lock(0);
        if (scanner_espnow_status) {
            lv_label_set_text(scanner_espnow_status, "Initializing...");
        }
        if (scanner_espnow_list) {
            lv_obj_clean(scanner_espnow_list);
        }
        bsp_display_unlock();

        // Init WiFi for promiscuous mode first
        if (!espnow_init_for_scan()) {
            bsp_display_lock(0);
            if (scanner_espnow_status) {
                lv_label_set_text(scanner_espnow_status, "Init failed - low memory");
            }
            bsp_display_unlock();
            scanner_switching_tabs = false;
            return;
        }

        // Now create task with smaller stack
        xTaskCreate(espnow_scanner_task, "espnow_scan", 4096, NULL, 3, &scanner_task_handle);
    }

    scanner_switching_tabs = false;
}

static void create_scanner_screen(void)
{
    scanner_screen_active = true;  // Mark screen as active before any UI creation

    bsp_display_lock(0);

    lv_obj_t *scr = lv_scr_act();

    // Main container (dark background, full screen)
    scanner_container = lv_obj_create(scr);
    lv_obj_set_size(scanner_container, 410, 502);
    lv_obj_center(scanner_container);
    lv_obj_set_style_bg_color(scanner_container, lv_color_hex(0x0a1628), 0);
    lv_obj_set_style_bg_opa(scanner_container, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(scanner_container, 0, 0);
    lv_obj_set_style_radius(scanner_container, 0, 0);
    lv_obj_set_style_pad_all(scanner_container, 0, 0);
    lv_obj_remove_flag(scanner_container, LV_OBJ_FLAG_SCROLLABLE);

    // Create tabview (tabs at top, moved down to clear rounded corners/battery)
    scanner_tabview = lv_tabview_create(scanner_container);
    lv_obj_set_size(scanner_tabview, 410, 425);  // Reduced height to fit
    lv_obj_align(scanner_tabview, LV_ALIGN_TOP_MID, 0, 35);  // Move down 35px from top
    lv_tabview_set_tab_bar_size(scanner_tabview, 72);  // Double height for readability
    lv_tabview_set_tab_bar_position(scanner_tabview, LV_DIR_TOP);

    // Style the tabview
    lv_obj_set_style_bg_color(scanner_tabview, lv_color_hex(0x0a1628), 0);
    lv_obj_set_style_border_width(scanner_tabview, 0, 0);

    // Style the tab bar
    lv_obj_t *tab_bar = lv_tabview_get_tab_bar(scanner_tabview);
    lv_obj_set_style_bg_color(tab_bar, lv_color_hex(0x142a4a), 0);
    lv_obj_set_style_pad_all(tab_bar, 2, 0);

    // Create tabs
    scanner_wifi_tab = lv_tabview_add_tab(scanner_tabview, LV_SYMBOL_WIFI " WiFi");
    scanner_bt_tab = lv_tabview_add_tab(scanner_tabview, LV_SYMBOL_BLUETOOTH " BT");
    scanner_espnow_tab = lv_tabview_add_tab(scanner_tabview, LV_SYMBOL_SHUFFLE " NOW");

    // Style each tab content area
    lv_obj_set_style_bg_color(scanner_wifi_tab, lv_color_hex(0x0a1628), 0);
    lv_obj_set_style_bg_color(scanner_bt_tab, lv_color_hex(0x0a1628), 0);
    lv_obj_set_style_bg_color(scanner_espnow_tab, lv_color_hex(0x0a1628), 0);
    lv_obj_set_style_pad_all(scanner_wifi_tab, 4, 0);
    lv_obj_set_style_pad_all(scanner_bt_tab, 4, 0);
    lv_obj_set_style_pad_all(scanner_espnow_tab, 4, 0);

    // === WiFi Tab Content ===
    scanner_wifi_status = lv_label_create(scanner_wifi_tab);
    lv_label_set_text(scanner_wifi_status, "Starting scan...");
    lv_obj_set_style_text_color(scanner_wifi_status, lv_color_hex(0x88aacc), 0);
    lv_obj_set_style_text_font(scanner_wifi_status, &lv_font_montserrat_12, 0);
    lv_obj_align(scanner_wifi_status, LV_ALIGN_TOP_MID, 0, 2);

    scanner_wifi_list = create_scanner_list(scanner_wifi_tab);

    // === Bluetooth Tab Content ===
    scanner_bt_status = lv_label_create(scanner_bt_tab);
#if CONFIG_BT_ENABLED
    lv_label_set_text(scanner_bt_status, "Tap tab to scan");
#else
    lv_label_set_text(scanner_bt_status, "BT not enabled in sdkconfig");
#endif
    lv_obj_set_style_text_color(scanner_bt_status, lv_color_hex(0x88aacc), 0);
    lv_obj_set_style_text_font(scanner_bt_status, &lv_font_montserrat_12, 0);
    lv_obj_align(scanner_bt_status, LV_ALIGN_TOP_MID, 0, 2);

    scanner_bt_list = create_scanner_list(scanner_bt_tab);

#if !CONFIG_BT_ENABLED
    // Placeholder message for BT tab (only when BT disabled)
    lv_obj_t *bt_placeholder = lv_label_create(scanner_bt_list);
    lv_label_set_text(bt_placeholder, "Enable CONFIG_BT_ENABLED\nin menuconfig to scan\nfor Bluetooth devices");
    lv_obj_set_style_text_color(bt_placeholder, lv_color_hex(0x3a5a7c), 0);
    lv_obj_set_style_text_font(bt_placeholder, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_align(bt_placeholder, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(bt_placeholder);
#endif

    // === ESP-NOW Tab Content ===
    scanner_espnow_status = lv_label_create(scanner_espnow_tab);
    lv_label_set_text(scanner_espnow_status, "Tap tab to scan");
    lv_obj_set_style_text_color(scanner_espnow_status, lv_color_hex(0x88aacc), 0);
    lv_obj_set_style_text_font(scanner_espnow_status, &lv_font_montserrat_12, 0);
    lv_obj_align(scanner_espnow_status, LV_ALIGN_TOP_MID, 0, 2);

    scanner_espnow_list = create_scanner_list(scanner_espnow_tab);

    // Add tab change handler
    lv_obj_add_event_cb(scanner_tabview, scanner_tab_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Add swipe handler to container (for back navigation)
    lv_obj_add_event_cb(scanner_container, global_event_cb, LV_EVENT_ALL, NULL);

    // Return button (lower-right corner)
    scanner_return_btn = lv_btn_create(scanner_container);
    lv_obj_set_size(scanner_return_btn, 80, 40);
    lv_obj_align(scanner_return_btn, LV_ALIGN_BOTTOM_RIGHT, -20, -18);
    lv_obj_set_style_bg_color(scanner_return_btn, lv_color_hex(0x1a3a5c), 0);
    lv_obj_set_style_bg_color(scanner_return_btn, lv_color_hex(0x2a5a8c), LV_STATE_PRESSED);
    lv_obj_add_event_cb(scanner_return_btn, scanner_return_btn_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_label = lv_label_create(scanner_return_btn);
    lv_label_set_text(btn_label, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(btn_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(btn_label, &lv_font_montserrat_12, 0);
    lv_obj_center(btn_label);

    bsp_display_unlock();

    // Reset state
    scanner_active_tab = 0;
    scanner_switching_tabs = false;

    // Show init status
    bsp_display_lock(0);
    if (scanner_wifi_status) {
        lv_label_set_text(scanner_wifi_status, "Initializing WiFi...");
    }
    bsp_display_unlock();

    // Init WiFi first (needs ~40KB heap, must happen before task stack allocation)
    if (!wifi_init_for_scan()) {
        bsp_display_lock(0);
        if (scanner_wifi_status) {
            lv_label_set_text(scanner_wifi_status, "WiFi init failed - low memory");
        }
        bsp_display_unlock();
        ESP_LOGE(TAG, "Scanner screen created but WiFi init failed");
        return;
    }

    // Start the WiFi scan task (WiFi already initialized, use smaller stack)
    xTaskCreate(wifi_scanner_task, "wifi_scan", 4096, NULL, 3, &scanner_task_handle);

    ESP_LOGI(TAG, "Scanner screen created with tabs, WiFi scan started");
}

// Destroy scanner screen and shutdown radios
static void destroy_scanner_screen(void)
{
    // Mark screen as inactive FIRST to stop all UI updates from scanner tasks
    scanner_screen_active = false;

    // Give scanner tasks time to notice the flag and stop UI updates
    vTaskDelay(pdMS_TO_TICKS(50));

    // Stop scan task if still running
    if (scanner_task_handle != NULL) {
        vTaskDelete(scanner_task_handle);
        scanner_task_handle = NULL;
        ESP_LOGI(TAG, "Scanner task stopped");
    }

    // Shutdown radios to free RAM
    wifi_scanner_shutdown();
#if CONFIG_BT_ENABLED
    ble_scanner_shutdown();
#endif
    espnow_scanner_shutdown();

    // Allow radio hardware to fully release resources before UI cleanup
    vTaskDelay(pdMS_TO_TICKS(100));

    bsp_display_lock(0);

    if (scanner_container) {
        lv_obj_del(scanner_container);
        scanner_container = NULL;
        scanner_return_btn = NULL;
        scanner_tabview = NULL;
        scanner_wifi_tab = NULL;
        scanner_bt_tab = NULL;
        scanner_espnow_tab = NULL;
        scanner_wifi_list = NULL;
        scanner_bt_list = NULL;
        scanner_espnow_list = NULL;
        scanner_wifi_status = NULL;
        scanner_bt_status = NULL;
        scanner_espnow_status = NULL;
        ESP_LOGI(TAG, "Scanner screen destroyed");
    }

    bsp_display_unlock();

    // Allow LVGL to complete cleanup before continuing
    vTaskDelay(pdMS_TO_TICKS(100));
}

// Create audio screen with spectrum analyzer
static void create_audio_screen(void)
{
    bsp_display_lock(0);

    lv_obj_t *scr = lv_scr_act();

    // Main container (black background, full screen)
    audio_container = lv_obj_create(scr);
    lv_obj_set_size(audio_container, 410, 502);
    lv_obj_center(audio_container);
    lv_obj_set_style_bg_color(audio_container, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(audio_container, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(audio_container, 0, 0);
    lv_obj_set_style_radius(audio_container, 0, 0);
    lv_obj_remove_flag(audio_container, LV_OBJ_FLAG_SCROLLABLE);

    // Title label
    lv_obj_t *title_label = lv_label_create(audio_container);
    lv_label_set_text(title_label, "Spectrum Analyzer");
    lv_obj_set_style_text_color(title_label, lv_color_hex(0x88aacc), 0);
    lv_obj_set_style_text_font(title_label, &lv_font_montserrat_18, 0);
    lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 40);

    // Add swipe handler to container
    lv_obj_add_event_cb(audio_container, global_event_cb, LV_EVENT_ALL, NULL);

    // Return button (lower-right corner)
    audio_return_btn = lv_btn_create(audio_container);
    lv_obj_set_size(audio_return_btn, 90, 45);
    lv_obj_align(audio_return_btn, LV_ALIGN_BOTTOM_RIGHT, -25, -25);
    lv_obj_set_style_bg_color(audio_return_btn, lv_color_hex(0x1a3a5c), 0);
    lv_obj_set_style_bg_color(audio_return_btn, lv_color_hex(0x2a5a8c), LV_STATE_PRESSED);
    lv_obj_add_event_cb(audio_return_btn, audio_return_btn_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_label = lv_label_create(audio_return_btn);
    lv_label_set_text(btn_label, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(btn_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(btn_label, &lv_font_montserrat_14, 0);
    lv_obj_center(btn_label);

    bsp_display_unlock();

    // Start the spectrum analyzer (creates canvas on audio_container)
    esp_err_t ret = spec_analyzer_start(audio_container);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start spectrum analyzer: %s", esp_err_to_name(ret));
        // Show error message
        bsp_display_lock(0);
        lv_obj_t *error_label = lv_label_create(audio_container);
        lv_label_set_text(error_label, "Audio init failed");
        lv_obj_set_style_text_color(error_label, lv_color_hex(0xff4444), 0);
        lv_obj_set_style_text_font(error_label, &lv_font_montserrat_16, 0);
        lv_obj_center(error_label);
        bsp_display_unlock();
    }

    ESP_LOGI(TAG, "Audio screen created with spectrum analyzer");
}

// Destroy audio screen
static void destroy_audio_screen(void)
{
    // Stop spectrum analyzer first (stops task and timer)
    spec_analyzer_stop();

    // Allow audio hardware and I2S to fully settle before freeing UI resources
    vTaskDelay(pdMS_TO_TICKS(100));

    bsp_display_lock(0);

    if (audio_container) {
        lv_obj_del(audio_container);
        audio_container = NULL;
        audio_return_btn = NULL;
        ESP_LOGI(TAG, "Audio screen destroyed");
    }

    bsp_display_unlock();

    // Allow LVGL to complete cleanup before continuing
    vTaskDelay(pdMS_TO_TICKS(100));
}

// Create apps screen (black background with return button, ready for app widgets)
static void create_apps_screen(void)
{
    bsp_display_lock(0);

    lv_obj_t *scr = lv_scr_act();

    // Main container (black background, full screen)
    apps_container = lv_obj_create(scr);
    lv_obj_set_size(apps_container, 410, 502);
    lv_obj_center(apps_container);
    lv_obj_set_style_bg_color(apps_container, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(apps_container, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(apps_container, 0, 0);
    lv_obj_set_style_radius(apps_container, 0, 0);
    lv_obj_remove_flag(apps_container, LV_OBJ_FLAG_SCROLLABLE);

    // Add swipe handler to container
    lv_obj_add_event_cb(apps_container, global_event_cb, LV_EVENT_ALL, NULL);

    // Clock icon button (time settings app) - top left area
    apps_clock_btn = lv_btn_create(apps_container);
    lv_obj_set_size(apps_clock_btn, 90, 90);  // Larger to fit bigger icon
    lv_obj_align(apps_clock_btn, LV_ALIGN_TOP_LEFT, 40, 60);
    lv_obj_set_style_bg_color(apps_clock_btn, lv_color_hex(0x1a3a5c), 0);
    lv_obj_set_style_bg_color(apps_clock_btn, lv_color_hex(0x2a5a8c), LV_STATE_PRESSED);
    lv_obj_set_style_radius(apps_clock_btn, 15, 0);
    lv_obj_add_event_cb(apps_clock_btn, apps_clock_btn_cb, LV_EVENT_CLICKED, NULL);

    // Clock icon using LVGL symbol (list icon)
    lv_obj_t *clock_icon = lv_label_create(apps_clock_btn);
    lv_label_set_text(clock_icon, LV_SYMBOL_LIST);
    lv_obj_set_style_text_color(clock_icon, lv_color_white(), 0);
    lv_obj_set_style_text_font(clock_icon, &lv_font_montserrat_48, 0);  // Doubled from 28
    lv_obj_center(clock_icon);

    // Return button (lower-right corner)
    apps_return_btn = lv_btn_create(apps_container);
    lv_obj_set_size(apps_return_btn, 90, 45);
    lv_obj_align(apps_return_btn, LV_ALIGN_BOTTOM_RIGHT, -25, -25);
    lv_obj_set_style_bg_color(apps_return_btn, lv_color_hex(0x1a3a5c), 0);
    lv_obj_set_style_bg_color(apps_return_btn, lv_color_hex(0x2a5a8c), LV_STATE_PRESSED);
    lv_obj_add_event_cb(apps_return_btn, apps_return_btn_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_label = lv_label_create(apps_return_btn);
    lv_label_set_text(btn_label, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(btn_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(btn_label, &lv_font_montserrat_14, 0);
    lv_obj_center(btn_label);

    bsp_display_unlock();

    ESP_LOGI(TAG, "Apps screen created with clock button");
}

// Destroy apps screen
static void destroy_apps_screen(void)
{
    bsp_display_lock(0);

    // Destroy time settings if active
    if (time_settings_screen) {
        time_settings_destroy(time_settings_screen);
        time_settings_screen = NULL;
        on_time_settings_screen = false;
        ESP_LOGI(TAG, "Time settings destroyed");
    }

    if (apps_container) {
        lv_obj_del(apps_container);
        apps_container = NULL;
        apps_return_btn = NULL;
        apps_clock_btn = NULL;
        ESP_LOGI(TAG, "Apps screen destroyed");
    }

    bsp_display_unlock();

    // Allow LVGL to complete cleanup before continuing
    vTaskDelay(pdMS_TO_TICKS(100));
}

// Transition from watchface to apps screen
static void transition_to_apps(void)
{
    ESP_LOGI(TAG, "Transitioning to Apps screen");

    imu_updates_paused = true;
    destroy_watchface_only();

    // Allow LVGL to complete pending operations before creating new screen
    vTaskDelay(pdMS_TO_TICKS(100));

    create_apps_screen();
    move_battery_to_foreground();
    current_screen = SCREEN_APPS;
    inactivity_counter = 0;

    ESP_LOGI(TAG, "Now on Apps screen");
}

// Transition from watchface to scanner screen
static void transition_to_scanner(void)
{
    // Block scanner access until startup WiFi/NTP sequence is complete
    if (!wifi_init_complete) {
        ESP_LOGW(TAG, "Scanner blocked - WiFi startup sequence still in progress");
        return;
    }

    ESP_LOGI(TAG, "Transitioning to Scanner screen");

    imu_updates_paused = true;
    destroy_watchface_only();

    // Allow LVGL to complete pending operations before creating new screen
    vTaskDelay(pdMS_TO_TICKS(100));

    create_scanner_screen();
    move_battery_to_foreground();
    current_screen = SCREEN_SCANNER;
    inactivity_counter = 0;

    ESP_LOGI(TAG, "Now on Scanner screen");
}

// Transition from watchface to audio screen
static void transition_to_audio(void)
{
    ESP_LOGI(TAG, "Transitioning to Audio screen");

    imu_updates_paused = true;
    destroy_watchface_only();

    // Allow LVGL to complete pending operations before creating new screen
    vTaskDelay(pdMS_TO_TICKS(100));

    create_audio_screen();
    move_battery_to_foreground();
    current_screen = SCREEN_AUDIO;
    inactivity_counter = 0;

    ESP_LOGI(TAG, "Now on Audio screen");
}

// Return from any app screen to watchface
static void nav_return_to_watchface(void)
{
    ESP_LOGI(TAG, "Returning to Watchface from screen %d", current_screen);

    // Keep IMU paused during transition to prevent race conditions
    // (will be resumed after watchface is fully created)

    // Destroy current app screen
    if (current_screen == SCREEN_SCANNER) {
        destroy_scanner_screen();
    } else if (current_screen == SCREEN_AUDIO) {
        destroy_audio_screen();
    } else if (current_screen == SCREEN_APPS) {
        destroy_apps_screen();
    }

    // Allow LVGL and hardware to settle before creating new screen
    vTaskDelay(pdMS_TO_TICKS(100));

    // Recreate watchface (while IMU is still paused)
    bsp_display_lock(0);
    lv_obj_t *scr = lv_scr_act();

    // Re-add global event handler for swipe gestures and long press
    // (was removed when previous screen was deleted)
    lv_obj_add_event_cb(scr, global_event_cb, LV_EVENT_ALL, NULL);

    switch (current_watchface_type) {
        case WATCHFACE_TYPE_PILOT:
            watchface = pilot_watchface_create(scr);
            if (!watchface) {
                // Fall back to simple watchface if pilot fails (low memory after scanner/audio)
                ESP_LOGW(TAG, "Pilot watchface failed after app exit, falling back to simple");
                watchface = simple_watchface_create(scr);
                if (watchface) {
                    current_watchface_type = WATCHFACE_TYPE_SIMPLE;
                }
            }
            break;
        case WATCHFACE_TYPE_SIMPLE:
        default:
            watchface = simple_watchface_create(scr);
            break;
    }
    bsp_display_unlock();

    // Update displays
    pcf85063a_get_time_date(&rtc_dev, &current_time);
    update_time_display();
    update_battery_display();

    // Ensure battery stays on top
    move_battery_to_foreground();

    current_screen = SCREEN_WATCHFACE;
    inactivity_counter = 0;

    // Resume IMU updates AFTER watchface is fully created and screen state is set
    // This prevents race conditions with the IMU task accessing an incomplete watchface
    imu_updates_paused = false;

    // Log heap status after returning to watchface
    ESP_LOGI(TAG, "Now on Watchface, heap free: %lu", esp_get_free_heap_size());
}

// ========================================================================
// END MULTI-SCREEN NAVIGATION
// ========================================================================

// ========================================================================
// START LVGL UI CREATION
// ========================================================================

// LVGL screen event callback - resets inactivity on screen touch events
// Also handles long press to open watchface selector and swipe gestures
static void global_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    // Reset inactivity counter on touch-related events
    if (code == LV_EVENT_PRESSED ||
        code == LV_EVENT_PRESSING ||
        code == LV_EVENT_CLICKED ||
        code == LV_EVENT_RELEASED) {
        inactivity_counter = 0;
    }

    // Long press opens watchface selector (only from watchface screen, not settings)
    if (code == LV_EVENT_LONG_PRESSED && !on_settings_screen && current_screen == SCREEN_WATCHFACE) {
        ESP_LOGI(TAG, "Long press detected - opening watchface selector");
        show_watchface_selector();
    }

    // Swipe gesture detection - skip if on settings screen
    if (on_settings_screen) {
        return;
    }

    if (code == LV_EVENT_PRESSED) {
        // Record swipe start point
        lv_indev_t *indev = lv_indev_active();
        if (indev != NULL) {
            lv_indev_get_point(indev, &swipe_start_point);
            tracking_swipe = true;
        }
    }
    else if (code == LV_EVENT_RELEASED && tracking_swipe) {
        tracking_swipe = false;

        // Get release point and calculate delta
        lv_point_t release_point;
        lv_indev_t *indev = lv_indev_active();
        if (indev != NULL) {
            lv_indev_get_point(indev, &release_point);

            int32_t delta_y = release_point.y - swipe_start_point.y;
            int32_t delta_x = release_point.x - swipe_start_point.x;

            // Check for vertical swipe (Y movement > X movement and exceeds threshold)
            if (abs(delta_y) > abs(delta_x) && abs(delta_y) > SWIPE_THRESHOLD_PX) {
                if (delta_y < 0) {
                    // Swipe UP
                    ESP_LOGI(TAG, "Swipe UP detected (delta_y=%ld)", (long)delta_y);
                    if (current_screen == SCREEN_WATCHFACE) {
                        transition_to_scanner();
                    } else if (current_screen == SCREEN_AUDIO) {
                        nav_return_to_watchface();
                    }
                } else {
                    // Swipe DOWN
                    ESP_LOGI(TAG, "Swipe DOWN detected (delta_y=%ld)", (long)delta_y);
                    if (current_screen == SCREEN_WATCHFACE) {
                        transition_to_audio();
                    } else if (current_screen == SCREEN_SCANNER) {
                        nav_return_to_watchface();
                    }
                }
            }
            // Check for horizontal swipe (X movement > Y movement and exceeds threshold)
            else if (abs(delta_x) > abs(delta_y) && abs(delta_x) > SWIPE_THRESHOLD_PX) {
                if (delta_x > 0) {
                    // Swipe RIGHT
                    ESP_LOGI(TAG, "Swipe RIGHT detected (delta_x=%ld)", (long)delta_x);
                    if (current_screen == SCREEN_WATCHFACE) {
                        transition_to_apps();
                    }
                } else {
                    // Swipe LEFT
                    ESP_LOGI(TAG, "Swipe LEFT detected (delta_x=%ld)", (long)delta_x);
                    if (current_screen == SCREEN_APPS) {
                        nav_return_to_watchface();
                    }
                }
            }
        }
    }
}

// ========================================================================
// INIT SCREEN - Shown during WiFi/NTP startup to minimize memory usage
// ========================================================================

// Create minimal init screen (black background + "Initializing..." text)
static void create_init_screen(void)
{
    bsp_display_lock(0);

    lv_obj_t *scr = lv_scr_act();

    // Black background
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    // Simple "Initializing..." label
    init_screen_label = lv_label_create(scr);
    lv_label_set_text(init_screen_label, "Initializing...");
    lv_obj_set_style_text_color(init_screen_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(init_screen_label, &lv_font_montserrat_30, 0);
    lv_obj_center(init_screen_label);

    init_screen_active = true;

    bsp_display_unlock();

    ESP_LOGI(TAG, "Init screen created (minimal memory footprint)");
}

// Update init screen status text
static void update_init_screen_status(const char *status)
{
    if (!init_screen_active || !init_screen_label) return;

    bsp_display_lock(0);
    lv_label_set_text(init_screen_label, status);
    bsp_display_unlock();
}

// Destroy init screen before creating watchface
static void destroy_init_screen(void)
{
    if (!init_screen_active) return;

    bsp_display_lock(0);

    if (init_screen_label) {
        lv_obj_del(init_screen_label);
        init_screen_label = NULL;
    }

    init_screen_active = false;

    bsp_display_unlock();

    ESP_LOGI(TAG, "Init screen destroyed, ready for watchface");
}

// ========================================================================
// END INIT SCREEN
// ========================================================================

// Create watchface UI based on current_watchface_type
static void create_ui(void)
{
    bsp_display_lock(0);

    lv_obj_t *scr = lv_scr_act();

    // Add global event handler to catch all touch events and long press
    lv_obj_add_event_cb(scr, global_event_cb, LV_EVENT_ALL, NULL);

    // Create the appropriate watchface based on saved selection
    switch (current_watchface_type) {
        case WATCHFACE_TYPE_PILOT:
            watchface = pilot_watchface_create(scr);
            if (watchface) {
                ESP_LOGI(TAG, "Pilot watchface created");
            } else {
                // Fall back to simple watchface if pilot fails (low memory)
                ESP_LOGW(TAG, "Pilot watchface failed, falling back to simple");
                watchface = simple_watchface_create(scr);
                if (watchface) {
                    current_watchface_type = WATCHFACE_TYPE_SIMPLE;
                    ESP_LOGI(TAG, "Simple watchface created as fallback");
                }
            }
            break;
        case WATCHFACE_TYPE_SIMPLE:
        default:
            watchface = simple_watchface_create(scr);
            ESP_LOGI(TAG, "Simple watchface created");
            break;
    }

    if (!watchface) {
        ESP_LOGE(TAG, "CRITICAL: Failed to create ANY watchface!");
    }

    bsp_display_unlock();

    // Create battery widget AFTER unlocking display (it has its own lock)
    create_battery_widget();
}

// Destroy UI to save compute when display is off
static void destroy_ui(void)
{
    bsp_display_lock(0);

    // Destroy watchface based on current type
    if (watchface) {
        switch (current_watchface_type) {
            case WATCHFACE_TYPE_PILOT:
                pilot_watchface_destroy((pilot_watchface_t *)watchface);
                break;
            case WATCHFACE_TYPE_SIMPLE:
            default:
                simple_watchface_destroy((simple_watchface_t *)watchface);
                break;
        }
        watchface = NULL;
        ESP_LOGI(TAG, "Watchface destroyed");
    }

    // Destroy battery widget (deleting container deletes all children)
    if (battery_container) {
        lv_obj_del(battery_container);
        battery_container = NULL;
        battery_body = NULL;
        battery_tip = NULL;
        battery_fill = NULL;
        battery_percent_label = NULL;
    }

    // Also delete usb_icon and charging_icon if they exist outside container
    if (usb_icon) {
        lv_obj_del(usb_icon);
        usb_icon = NULL;
    }
    if (charging_icon) {
        lv_obj_del(charging_icon);
        charging_icon = NULL;
    }

    // Clean all remaining children from the screen to ensure fresh state
    lv_obj_t *scr = lv_scr_act();
    if (scr) {
        lv_obj_clean(scr);
        ESP_LOGI(TAG, "Screen cleaned");
    }

    bsp_display_unlock();
}

// ========================================================================
// END LVGL UI CREATION 
// ========================================================================


extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Smartwatch Starting");

    // Pulling saved settings from NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

//---------------------------------------------
// LOAD PREVIOUS BOOT MEMORY STATS - REMOVE FROM FINAL VERSION
//---------------------------------------------

    #if ENABLE_MEMORY_STATS_NVS
    load_previous_memory_stats();
    #endif

//---------------------------------------------
// END REMOVE FROM FINAL VERSION
//---------------------------------------------

    // Set Timezone
    setenv("TZ", TIMEZONE, 1);
    tzset();
    ESP_LOGI(TAG, "Timezone set to: %s", TIMEZONE);


    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("wifi_init", ESP_LOG_WARN);
    esp_log_level_set("phy_init", ESP_LOG_WARN);

    ESP_LOGI(TAG, "Initializing I2C bus...");
    esp_err_t i2c_ret = bsp_i2c_init();
    if (i2c_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2C: %s", esp_err_to_name(i2c_ret));
    }

    // Init PMU on I2C bus - this enables ALDO2 for display power
    ESP_LOGI(TAG, "Initializing PMU (enables display power)...");
    ESP_ERROR_CHECK(pmu_i2c_init());
    ESP_ERROR_CHECK(pmu_init());
    ESP_LOGI(TAG, "PMU initialized - display power (ALDO2) enabled");

    // 500ms delay for PMU/ALDO2 to stabilize before any WiFi/radio operations
    // This prevents I2C bus contention between PMU reads and radio init
    vTaskDelay(pdMS_TO_TICKS(500));

    // NOTE: bsp_display_start() is DELAYED until after WiFi/NTP completes
    // This frees ~40KB+ RAM for WiFi stack during startup
    // Screen stays black during WiFi sync (a few seconds max)

    // Load saved watchface preference from NVS (will be used after WiFi shuts down)
    current_watchface_type = load_watchface_type_from_nvs();
    ESP_LOGI(TAG, "Watchface type loaded: %d (%s) - display will start after WiFi",
             current_watchface_type,
             current_watchface_type == WATCHFACE_TYPE_PILOT ? "Pilot" : "Simple");

    // Load saved time mode preference from NVS (determines if WiFi/NTP runs at startup)
    time_mode_is_manual = load_time_mode_from_nvs();
    ESP_LOGI(TAG, "Time mode loaded: %s - %s",
             time_mode_is_manual ? "Manual" : "Auto",
             time_mode_is_manual ? "skipping WiFi/NTP" : "will sync with NTP");

    // Init RTC (I2C already initialized above)
    i2c_master_bus_handle_t i2c_handle = bsp_i2c_get_handle();
    esp_err_t rtc_ret = pcf85063a_init(&rtc_dev, i2c_handle, PCF85063A_ADDRESS);
    if (rtc_ret == ESP_OK) {
        ESP_LOGI(TAG, "RTC initialized");
        // Read initial time (will be displayed once watchface is created)
        read_rtc_time();
    } else {
        ESP_LOGE(TAG, "RTC init failed: %s - time will show 00:00:00", esp_err_to_name(rtc_ret));
    }

    // Init IMU (QMI8658) on shared I2C bus
    ESP_LOGI(TAG, "Initializing IMU...");
    i2c_master_bus_handle_t bus_handle = bsp_i2c_get_handle();
    if (qmi8658_init(&imu_dev, bus_handle, QMI8658_ADDRESS_HIGH) == ESP_OK) {
        qmi8658_set_accel_range(&imu_dev, QMI8658_ACCEL_RANGE_8G);
        qmi8658_set_accel_odr(&imu_dev, QMI8658_ACCEL_ODR_62_5HZ);
        qmi8658_set_accel_unit_mps2(&imu_dev, true);
        ESP_LOGI(TAG, "IMU initialized: 8G range, 62.5Hz ODR");
    } else {
        ESP_LOGE(TAG, "IMU init failed!");
    }

     // Init NTP (before WiFi starts)
    ntp_init();

    // Init raw touch interrupt for reliable wake detection
    init_touch_wake_interrupt();

    // Mark startup as complete and signal sequencer
    startup_complete = true;
    ESP_LOGI(TAG, "Hardware initialization complete");

    // Enable CPU frequency scaling (160MHz active, 10MHz idle) for power savings
    init_power_management();

    // Start startup sequencer (waits 1s, then WiFi init/sync/shutdown)
    xTaskCreate(startup_complete_task, "startup_seq", 4096, NULL, 3, NULL);

    // Start time display task (updates every second)
    // Stack: 1948 bytes (~74% used based on high water mark testing)
    xTaskCreate(time_display_task, "time_display", 1948, NULL, 5, &time_display_task_handle);

    // Start IMU task (updates sub-dials + flip-to-wake detection)
    // Stack: 2460 bytes (~77% used based on high water mark testing)
    xTaskCreate(imu_display_task, "imu_wake", 2460, NULL, 5, NULL);

    // Start PMU task (battery monitoring)
    // Stack: 2400 bytes (~70% used based on high water mark testing)
    xTaskCreate(pmu_hander_task, "App/pwr", 2400, NULL, 5, &pmu_task_handle);

    // Start WiFi scan task (disabled, deletes itself immediately)
    xTaskCreate(wifi_scan_task, "wifi_scan", 4096, NULL, 3, &wifi_task_handle);

    // Start sleep management (backlight timer with flip/touch wake)
    // Stack: 2560 bytes (~66% used based on high water mark testing)
    // Stages: 20s dim -> 30s off + UI destroyed -> 40s ALDO2 off (max power save)
    xTaskCreate(sleep_management_task, "sleep_mgmt", 2048, NULL, 4, NULL);  // Reduced from 2560






    // NOTE: bsp_display_start() is called in startup_complete_task after WiFi shutdown
    // This maximizes available RAM for WiFi stack during NTP sync
    ESP_LOGI(TAG, "app_main complete - display will start after WiFi sequence");
}