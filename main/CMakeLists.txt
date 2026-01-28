idf_component_register(
    SRCS main.cpp port_axp2101.cpp Spec_Analyzer.c
    INCLUDE_DIRS .
    REQUIRES nvs_flash
             esp_wifi
             esp_event
             esp_netif
             simple_watchface
             pilot_watchface
             XPowersLib
             espressif__esp_lcd_touch
             espressif__esp_lvgl_port
             bsp_extra
    PRIV_REQUIRES bt
                  time_settings
)

idf_component_get_property(LVGL_LIB lvgl__lvgl COMPONENT_LIB)
target_compile_options(
    ${LVGL_LIB}
    PRIVATE
        -DLV_LVGL_H_INCLUDE_SIMPLE
)

# Note: M_PI is defined at top of main.cpp before includes to prevent
# redefinition warnings from qmi8658.h
