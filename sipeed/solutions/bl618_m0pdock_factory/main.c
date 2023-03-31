/**
 * @file main.c
 * @brief
 *
 * Copyright (c) 2021 Bouffalolab team
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 *
 */
#include "board.h"
#include "bflb_gpio.h"
#include "bflb_l1c.h"
#include "bflb_mtimer.h"

#include "lv_conf.h"
#include "lvgl.h"

#include "lv_port_disp.h"
#include "lv_port_indev.h"

#include "demos/demo.h"

#include "lcd.h"

#include "bflb_adc.h"
#include "bflb_cam.h"
#include "bflb_dma.h"
#include "csi_rv32_gcc.h"
#include "image_sensor.h"

#define DBG_TAG "main"
#include "log.h"

static struct bflb_device_s *cam0;
static struct bflb_device_s *adc;

static int filesystem_init(void);
static void cam_init(void);
static void adc_init(void);

/* lvgl log cb */
static void lv_log_print_g_cb(const char *buf)
{
    // printf("[LVGL] %s\r\n", buf);
}

int main(void)
{
    board_init();

    label_tfcard_state_update(!filesystem_init());

    struct bflb_device_s *gpio;
    gpio = bflb_device_get_by_name("gpio");
    /* backlight pin */
    bflb_gpio_init(gpio, GPIO_PIN_2, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
    bflb_gpio_set(gpio, GPIO_PIN_2);

    /* lvgl init */
    lv_log_register_print_cb(lv_log_print_g_cb);
    lv_init();
    lv_port_disp_init();
    lv_port_indev_init();

    demo();

    /* ADC_CH0 */
    bflb_gpio_init(gpio, GPIO_PIN_20, GPIO_ANALOG | GPIO_SMT_EN | GPIO_DRV_0);
    /* ADC_CH3 */
    bflb_gpio_init(gpio, GPIO_PIN_3, GPIO_ANALOG | GPIO_SMT_EN | GPIO_DRV_0);
    /* adc init */
    adc_init();

    /* DVP0 GPIO init */
    /* I2C GPIO */
    // bflb_gpio_init(gpio, GPIO_PIN_0, GPIO_FUNC_I2C0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    // bflb_gpio_init(gpio, GPIO_PIN_1, GPIO_FUNC_I2C0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);

    /* Power down GPIO */
    bflb_gpio_init(gpio, GPIO_PIN_16, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_reset(gpio, GPIO_PIN_16);

    /* MCLK GPIO */
    bflb_gpio_init(gpio, GPIO_PIN_6, GPIO_FUNC_CLKOUT | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);

    /* DVP0 GPIO */
    bflb_gpio_init(gpio, GPIO_PIN_24, GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_init(gpio, GPIO_PIN_25, GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_init(gpio, GPIO_PIN_26, GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_init(gpio, GPIO_PIN_27, GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_init(gpio, GPIO_PIN_28, GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_init(gpio, GPIO_PIN_29, GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_init(gpio, GPIO_PIN_30, GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_init(gpio, GPIO_PIN_31, GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_init(gpio, GPIO_PIN_32, GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_init(gpio, GPIO_PIN_33, GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_init(gpio, GPIO_PIN_34, GPIO_FUNC_CAM | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    cam_init();

    LOG_I("lvgl success\r\n");

    while (1) {
        lv_task_handler();
        bflb_mtimer_delay_ms(1);
    }
}

#include "ff.h"
#include "fatfs_diskio_register.h"

static int filesystem_init(void)
{
    static FATFS fs;
    static __attribute((aligned(8))) uint32_t workbuf[4 * 1024];

    MKFS_PARM fs_para = {
        .fmt = FM_FAT32,     /* Format option (FM_FAT, FM_FAT32, FM_EXFAT and FM_SFD) */
        .n_fat = 1,          /* Number of FATs */
        .align = 0,          /* Data area alignment (sector) */
        .n_root = 1,         /* Number of root directory entries */
        .au_size = 512 * 32, /* Cluster size (byte) */
    };

    FRESULT ret;

    board_sdh_gpio_init();

    fatfs_sdh_driver_register();

    ret = f_mount(&fs, "/sd", 1);

    if (ret == FR_NO_FILESYSTEM) {
        LOG_W("No filesystem yet, try to be formatted...\r\n");

        ret = f_mkfs("/sd", &fs_para, workbuf, sizeof(workbuf));

        if (ret != FR_OK) {
            LOG_F("fail to make filesystem\r\n");
            return ret;
        }

        if (ret == FR_OK) {
            LOG_I("done with formatting.\r\n");
            LOG_I("first start to unmount.\r\n");
            ret = f_mount(NULL, "/sd", 1);
            LOG_I("then start to remount.\r\n");
        }
    } else if (ret != FR_OK) {
        LOG_F("fail to mount filesystem,error= %d\r\n", ret);
        LOG_F("SD card might fail to initialise.\r\n");
        return ret;
    } else {
        LOG_D("Succeed to mount filesystem\r\n");
    }

    if (ret == FR_OK) {
        LOG_I("FileSystem cluster size:%d-sectors (%d-Byte)\r\n", fs.csize, fs.csize * 512);
    }

    return ret;
}

static void cam_isr(int irq, void *arg);

static void cam_init(void)
{
    struct bflb_cam_config_s cam_config;
    struct image_sensor_config_s *sensor_config;
    struct bflb_device_s *i2c0;

    i2c0 = bflb_device_get_by_name("i2c0");
    cam0 = bflb_device_get_by_name("cam0");

    if (image_sensor_scan(i2c0, &sensor_config)) {
        LOG_I("\r\nSensor name: %s\r\n", sensor_config->name);
    } else {
        LOG_E("\r\nError! Can't identify sensor!\r\n");
        cam0 = NULL;
        return;
    }

    bflb_cam_int_mask(cam0, CAM_INTMASK_NORMAL, false);
    bflb_irq_attach(cam0->irq_num, cam_isr, NULL);
    bflb_irq_enable(cam0->irq_num);

    memcpy(&cam_config, sensor_config, IMAGE_SENSOR_INFO_COPY_SIZE);
    cam_config.with_mjpeg = false;
    cam_config.output_format = CAM_OUTPUT_FORMAT_AUTO;
    static lv_color_t cam_buffer[4][320 * 240] __section(".psmram_data");
    cam_config.output_bufaddr = (uint32_t)(uintptr_t)(void *)cam_buffer;
    cam_config.output_bufsize = sizeof(cam_buffer);

    bflb_cam_init(cam0, &cam_config);
    bflb_cam_start(cam0);

    // bflb_cam_stop(cam0);
}

#define UPDATE_FREQ       2
#define TEST_COUNT        64
#define TEST_ADC_CHANNELS 2

static ATTR_NOCACHE_NOINIT_RAM_SECTION uint32_t adc_raw_data[2][TEST_ADC_CHANNELS * TEST_COUNT];
static void dma0_ch0_isr(void *arg);

static void adc_init(void)
{
    struct bflb_adc_channel_s chan[] = {
        { .pos_chan = ADC_CHANNEL_0,
          .neg_chan = ADC_CHANNEL_GND },
        { .pos_chan = ADC_CHANNEL_3,
          .neg_chan = ADC_CHANNEL_GND },
    };

    adc = bflb_device_get_by_name("adc");

    /**
     *  adc clock = XCLK / 2 / 20 / 64(14B) = 15.625K
     */
    struct bflb_adc_config_s cfg;
    cfg.clk_div = ADC_CLK_DIV_20;
    cfg.scan_conv_mode = true;
    cfg.continuous_conv_mode = true;
    cfg.differential_mode = false;
    cfg.resolution = ADC_RESOLUTION_14B;
    cfg.vref = ADC_VREF_2P0V;

    bflb_adc_init(adc, &cfg);
    bflb_adc_channel_config(adc, chan, sizeof(chan) / sizeof(chan[0]));
    bflb_adc_link_rxdma(adc, true);

    struct bflb_device_s *dma0_ch0;
    dma0_ch0 = bflb_device_get_by_name("dma0_ch0");

    struct bflb_dma_channel_config_s config;

    config.direction = DMA_PERIPH_TO_MEMORY;
    config.src_req = DMA_REQUEST_ADC;
    config.dst_req = DMA_REQUEST_NONE;
    config.src_addr_inc = DMA_ADDR_INCREMENT_DISABLE;
    config.dst_addr_inc = DMA_ADDR_INCREMENT_ENABLE;
    config.src_burst_count = DMA_BURST_INCR1;
    config.dst_burst_count = DMA_BURST_INCR1;
    config.src_width = DMA_DATA_WIDTH_32BIT;
    config.dst_width = DMA_DATA_WIDTH_32BIT;
    bflb_dma_channel_init(dma0_ch0, &config);

    bflb_dma_channel_irq_attach(dma0_ch0, dma0_ch0_isr, NULL);

    static struct bflb_dma_channel_lli_pool_s lli[20]; /* max trasnfer size 4064 * 20 */
    static struct bflb_dma_channel_lli_transfer_s transfers[2];

    transfers[0].src_addr = (uint32_t)DMA_ADDR_ADC_RDR;
    transfers[0].dst_addr = (uint32_t)adc_raw_data[0];
    transfers[0].nbytes = sizeof(adc_raw_data[0]);

    transfers[1].src_addr = (uint32_t)DMA_ADDR_ADC_RDR;
    transfers[1].dst_addr = (uint32_t)adc_raw_data[1];
    transfers[1].nbytes = sizeof(adc_raw_data[1]);

    int used_count = bflb_dma_channel_lli_reload(dma0_ch0, lli, 20, transfers, 2);
    bflb_dma_channel_lli_link_head(dma0_ch0, lli, used_count);
    bflb_dma_channel_start(dma0_ch0);

    // bflb_adc_start_conversion(adc);

    bflb_adc_stop_conversion(adc);
}

static void cam_isr(int irq, void *arg)
{
    uint16_t *pic_addr;
    uint32_t pic_size;
    // static volatile uint32_t cam_int_cnt = 0;

    bflb_cam_int_clear(cam0, CAM_INTCLR_NORMAL);
    pic_size = bflb_cam_get_frame_info(cam0, (void *)&pic_addr);
    bflb_cam_pop_one_frame(cam0);
    // printf("CAM interrupt, pop picture %d: 0x%08x, len: %d\r\n", cam_int_cnt++, (uint32_t)pic_addr, pic_size);
    for (size_t i = 0; i < pic_size / sizeof(uint16_t); i++) {
        pic_addr[i] = __bswap16(pic_addr[i]);
    }
    canvas_cam_update(pic_addr);
}

static void dma0_ch0_isr(void *arg)
{
    static uint32_t dma_tc_flag0 = 0;
    dma_tc_flag0++;
    // printf("[%d]tc done\r\n", dma_tc_flag0);

    struct bflb_adc_result_s result[TEST_ADC_CHANNELS * TEST_COUNT];
    bflb_adc_parse_result(adc, adc_raw_data[!(dma_tc_flag0 & 0x1)], result, TEST_ADC_CHANNELS * TEST_COUNT);

    int16_t results_temp[TEST_COUNT];
    uint32_t btn_adc_val_avg = 0;
    for (size_t j = 0, k = 0; j < TEST_ADC_CHANNELS * TEST_COUNT; j++) {
        // printf("raw data:%08x\r\n", adc_raw_data[!(dma_tc_flag0 & 0x1)][j]);
        // ADC_CHANNEL_0 min:923mv nor:952mv max:980mv
        // printf("pos chan %d,%d mv \r\n", result[j].pos_chan, result[j].millivolt);
        if (ADC_CHANNEL_3 == result[j].pos_chan) {
            btn_adc_val_avg += result[j].millivolt;
        } else if (ADC_CHANNEL_0 == result[j].pos_chan) {
            results_temp[k++] = result[j].millivolt; // - 952;
        }
    }
    btn_adc_val_avg /= TEST_COUNT;

    chart_mic_append_data(results_temp, TEST_COUNT);
    label_adc_btn_update(btn_adc_val_avg);
}

void btn_cam_event_handled(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    lv_obj_t *btn = e->target;
    lv_label_t *label = (lv_label_t *)lv_obj_get_child(btn, 0);

    bool is_btn_checked = (lv_obj_get_state(btn) & LV_STATE_CHECKED);

    if (code == LV_EVENT_VALUE_CHANGED) {
        // LV_LOG_USER("%s checked %u", label->text, is_btn_checked);
        if (cam0) {
            (void (*[])(struct bflb_device_s *)){ bflb_cam_stop, bflb_cam_start }[is_btn_checked](cam0);
        }
    }
}

void btn_adc_event_handled(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    lv_obj_t *btn = e->target;
    lv_label_t *label = (lv_label_t *)lv_obj_get_child(btn, 0);

    bool is_btn_checked = (lv_obj_get_state(btn) & LV_STATE_CHECKED);

    if (code == LV_EVENT_VALUE_CHANGED) {
        // LV_LOG_USER("%s checked %u", label->text, is_btn_checked);
        if (adc) {
            (void (*[])(struct bflb_device_s *)){ bflb_adc_stop_conversion, bflb_adc_start_conversion }[is_btn_checked](adc);
            if (!is_btn_checked) {
                lv_label_set_text(label, "ADC");
            }
        }
    }
}