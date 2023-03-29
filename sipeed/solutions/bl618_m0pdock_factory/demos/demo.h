#pragma once

#include "lvgl.h"

void demo(void);

void chart_mic_append_data(int16_t *data, uint16_t len);
void label_adc_btn_update(uint16_t val);