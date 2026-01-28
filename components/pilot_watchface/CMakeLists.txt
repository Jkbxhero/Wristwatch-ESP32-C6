# CMakeLists.txt for pilot_watchface component
# Add this to your project's components directory

idf_component_register(
    SRCS 
        "pilot_watchface.c"
    INCLUDE_DIRS 
        "."
    REQUIRES 
        lvgl__lvgl
)
