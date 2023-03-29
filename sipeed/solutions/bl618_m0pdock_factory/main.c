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

static struct bflb_device_s *cam0;
static struct bflb_device_s *adc;

static void adc_init(void);

/* lvgl log cb */
static void lv_log_print_g_cb(const char *buf)
{
    printf("[LVGL] %s\r\n", buf);
}

int main(void)
{
    board_init();

    struct bflb_device_s *gpio;
    gpio = bflb_device_get_by_name("gpio");
    /* backlight pin */
    bflb_gpio_init(gpio, GPIO_PIN_2, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);
    bflb_gpio_set(gpio, GPIO_PIN_2);

    /* ADC_CH0 */
    bflb_gpio_init(gpio, GPIO_PIN_20, GPIO_ANALOG | GPIO_SMT_EN | GPIO_DRV_0);
    /* ADC_CH3 */
    bflb_gpio_init(gpio, GPIO_PIN_3, GPIO_ANALOG | GPIO_SMT_EN | GPIO_DRV_0);
    /* adc init */
    adc_init();

    /* lvgl init */
    lv_log_register_print_cb(lv_log_print_g_cb);
    lv_init();
    lv_port_disp_init();
    lv_port_indev_init();

    demo();

    printf("lvgl success\r\n");

    while (1) {
        lv_task_handler();
        bflb_mtimer_delay_ms(1);
    }
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
     *  adc clock = XCLK /  2 / 32 / 128 (14B) =  4.882K 
     *                             /  60 (12B) = 10.38276K
     *  adc clock = XCLK / 1 /  20 / 128 (14B) = 15.625K
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

    bflb_adc_start_conversion(adc);

    // bflb_adc_stop_conversion(adc);
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