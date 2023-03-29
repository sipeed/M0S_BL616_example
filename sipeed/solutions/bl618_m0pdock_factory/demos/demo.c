#include "demo.h"

static lv_obj_t *canvas_cam;
static lv_obj_t *chart_mic;
static lv_obj_t *label_adc_btn;
static lv_obj_t *led_adc_btn[2];

static lv_coord_t ecg_sample[160];
static uint16_t label_adc_btn_val;

static lv_obj_t *canvas_cam_create(lv_obj_t *parent);
static lv_obj_t *chart_mic_create(lv_obj_t *parent);
static lv_obj_t *label_adc_btn_create(lv_obj_t *parent);

void demo(void)
{
    canvas_cam = canvas_cam_create(lv_scr_act());
    chart_mic = chart_mic_create(lv_scr_act());
    label_adc_btn = label_adc_btn_create(canvas_cam);

    lv_obj_t *led;
    led = lv_led_create(canvas_cam);
    led_adc_btn[0] = led;
    lv_obj_align(led, LV_ALIGN_TOP_LEFT, 0, 0);

    led = lv_led_create(canvas_cam);
    led_adc_btn[1] = led;
    lv_obj_align(led, LV_ALIGN_TOP_RIGHT, 0, 0);
}

void chart_mic_append_data(int16_t *data, uint16_t len)
{
    uint32_t pcnt = sizeof(ecg_sample) / sizeof(ecg_sample[0]);
    if (!chart_mic || !data || !len) {
        return;
    }
    if (len > pcnt)
        len = pcnt;

    lv_obj_t *chart = chart_mic;
    lv_chart_set_point_count(chart, 0);

    // memcpy(ecg_sample, ecg_sample + len, (pcnt - len) * sizeof(lv_coord_t));
    // memcpy(ecg_sample + pcnt - len, data, len * sizeof(lv_coord_t));
    for (uint32_t i = 0; i < pcnt - len; i++) {
        ecg_sample[i] = ecg_sample[i + len];
    }
    for (uint32_t i = 0; i < len; i++) {
        ecg_sample[pcnt - len + i] = data[i];
    }

    lv_chart_set_point_count(chart, pcnt);
}

void label_adc_btn_update(uint16_t val)
{
    if (!label_adc_btn) {
        return;
    }

    label_adc_btn_val = val;
    lv_label_set_text_fmt(label_adc_btn, "%-4u mv", label_adc_btn_val);

    bool btn_clicked[2];

    bool both_clicked = val < 31;
    if (both_clicked) {
        btn_clicked[0] = true;
        btn_clicked[1] = true;
    } else {
        btn_clicked[0] = val >= 700 && val < 800;
        btn_clicked[1] = val >= 31 && val < 50;
    }

    bool led_states[2];
    for (uint32_t i = 0; i < 2; i++) {
        led_states[i] = btn_clicked[i];
        // ((void (*[])(lv_obj_t *)){ lv_led_off, lv_led_on }[led_states[i]])(led_adc_btn[i]);
        lv_led_set_color(led_adc_btn[i], (lv_color_t[]){ lv_color_black(), lv_color_make(0xff, 0x00, 0x00) }[led_states[i]]);
    }
}

static lv_obj_t *canvas_cam_create(lv_obj_t *parent)
{
    lv_obj_t *canvas;
    canvas = lv_obj_create(parent);
    lv_obj_set_size(canvas, 320, 240);
    lv_obj_align(canvas, LV_ALIGN_TOP_MID, 0, 0);

    return canvas;
}

static lv_obj_t *chart_mic_create(lv_obj_t *parent)
{
    /*Create a chart*/
    lv_obj_t *chart;
    chart = lv_chart_create(parent);
    lv_obj_set_size(chart, 320, 80);
    lv_obj_align(chart, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 2000);

    /*Do not display points on the data*/
    lv_obj_set_style_size(chart, 0, LV_PART_INDICATOR);

    lv_chart_series_t *ser = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
    LV_ASSERT(ser == lv_chart_get_series_next(chart, NULL));

    uint32_t pcnt = sizeof(ecg_sample) / sizeof(ecg_sample[0]);
    lv_chart_set_point_count(chart, pcnt);
    lv_chart_set_ext_y_array(chart, lv_chart_get_series_next(chart, NULL), (lv_coord_t *)ecg_sample);

    lv_chart_set_zoom_x(chart, LV_IMG_ZOOM_NONE);
    lv_chart_set_zoom_y(chart, LV_IMG_ZOOM_NONE);

    return chart;
}

static lv_obj_t *label_adc_btn_create(lv_obj_t *parent)
{
    lv_obj_t *label;
    label = lv_label_create(parent);

    lv_obj_set_width(label, 100);
    lv_obj_align_to(label, parent, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_color(label, lv_color_make(0xff, 0x00, 0x00), 0);

    return label;
}