#ifndef SPEC_ANALYZER_H
#define SPEC_ANALYZER_H

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize and start the spectrum analyzer
 * @param parent The LVGL parent object to create the canvas on
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t spec_analyzer_start(lv_obj_t *parent);

/**
 * Stop and cleanup the spectrum analyzer
 */
void spec_analyzer_stop(void);

#ifdef __cplusplus
}
#endif

#endif // SPEC_ANALYZER_H
