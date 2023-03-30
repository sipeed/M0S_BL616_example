#pragma once

#include "lvgl.h"

void demo(void);

void canvas_cam_update(void *pic_addr);
void chart_mic_append_data(int16_t *data, uint16_t len);
void label_adc_btn_update(uint16_t val);

extern void btn_cam_event_handled(lv_event_t *e);
extern void btn_adc_event_handled(lv_event_t *e);