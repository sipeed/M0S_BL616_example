/**
  ******************************************************************************
  * @file    camera_used_as_la.c
  * @version V1.0
  * @date
  * @brief   This file is the peripheral case c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2020 Bouffalo Lab</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of Bouffalo Lab nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include "bflb_platform.h"
#include "image_sensor.h"
#include "image_sensor_port.h"
#include "bl616_pwm.h"
#include "bl616_glb.h"
#include "bl616_glb_gpio.h"
#include "bl616_gpio.h"

#define CAM_USE_ID    CAM0_ID
#define FRAME_SIZE    (4 * 1024)
#define BUFF_SIZE     (64 * 1024)
uint8_t buff[BUFF_SIZE];
#define PIN_CTRL      GLB_GPIO_PIN_30 /* connect this pin to VSYNC and HSYNC */

uint8_t *pictureArr[BUFF_SIZE / FRAME_SIZE];
uint32_t lengthArr[BUFF_SIZE / FRAME_SIZE];
uint32_t idx = 0;

void CameraInit(void)
{
    GLB_GPIO_Cfg_Type cfg;

    CAM_CFG_Type cameraCfg = {
        .swMode = CAM_SW_MODE_MANUAL,
        .swIntCnt = 0,
        .pixWidth = CAM_PIX_DATA_BIT_16,
        .dropMode = CAM_DROP_NONE,
        .linePol = CAM_LINE_ACTIVE_POLARITY_HIGH,
        .framePol = CAM_FRAME_ACTIVE_POLARITY_HIGH,
        .camSensorMode = CAM_SENSOR_MODE_V_AND_H,
        .burstType = CAM_BURST_TYPE_INCR16,
        .waitCount = 0x40,
        .memStart = 0,
        .memSize = 0,
        .frameSize = FRAME_SIZE,
    };

    CAM_DFE_Cfg_Type dfeCfg = {
        .hSyncLevel = CAM_DFE_SYNC_ACTIVE_HIGH,
        .vSyncLevel = CAM_DFE_SYNC_ACTIVE_HIGH,
        .dataOrder = CAM_DFE_BYTE_LOWER,
        .fifoThreshold = 2400,
    };

    GLB_GPIO_Type gpioPins[] = {GLB_GPIO_PIN_0, GLB_GPIO_PIN_1, GLB_GPIO_PIN_3, GLB_GPIO_PIN_10, GLB_GPIO_PIN_11, GLB_GPIO_PIN_12,
                                GLB_GPIO_PIN_13, GLB_GPIO_PIN_14, GLB_GPIO_PIN_15, GLB_GPIO_PIN_16, GLB_GPIO_PIN_17};
    GLB_GPIO_Func_Init(GPIO_FUN_CAM, gpioPins, sizeof(gpioPins)/sizeof(gpioPins[0]));

    //REFCLK
    GLB_Set_CAM_CLK(ENABLE, GLB_CAM_CLK_WIFIPLL_96M, 0);
    GLB_Set_Chip_Clock_Out3_Sel(GLB_CHIP_CLK_OUT_3_CAM_REF_CLK);
    cfg.gpioPin = GLB_GPIO_PIN_27; /* connect this pin to pix_clk pin */
    cfg.gpioFun = GPIO_FUN_CLOCK_OUT;
    GLB_GPIO_Init(&cfg);

    CAM_DFE_Init(&dfeCfg);
    CAM_8_Bit_Byte_Select(CAM_USE_ID,CAM_8_BIT_SELECT_LOWER);

    cameraCfg.memStart = (uint32_t)buff,
    cameraCfg.memSize = BUFF_SIZE,
    CAM_Disable(CAM_USE_ID);
    CAM_Init(CAM_USE_ID, &cameraCfg);
    CAM_Enable(CAM_USE_ID);
    CAM_DFE_Enable();
}

void SampleVStop(void)
{
    GLB_GPIO_Cfg_Type cfg = {
        .gpioMode = GPIO_MODE_OUTPUT,
        .pullType = GPIO_PULL_DOWN,
        .gpioPin = PIN_CTRL,
        .gpioFun = GPIO_FUN_GPIO,
    };
    GLB_GPIO_Init(&cfg);
    GLB_GPIO_Write(PIN_CTRL, 0);
    GLB_GPIO_Output_Enable(PIN_CTRL);
}

void SampleVStart(void)
{
    GLB_GPIO_Cfg_Type cfg = {
        .gpioMode = GPIO_MODE_OUTPUT,
        .pullType = GPIO_PULL_DOWN,
        .gpioPin = PIN_CTRL,
        .gpioFun = GPIO_FUN_GPIO,
    };
    GLB_GPIO_Init(&cfg);
    GLB_GPIO_Write(PIN_CTRL, 1);
    GLB_GPIO_Output_Enable(PIN_CTRL);
}

int main(void)
{
    CAM_Frame_Info info;
    uint8_t* picture;
    uint32_t length;
    bflb_platform_init(0);
    MSG("camera used as logic analyzer:\r\n");
    SampleVStop();
    arch_delay_ms(2);
    CameraInit();
    arch_delay_ms(10);
    SampleVStart();
    while (1) {
        do {
            ARCH_MemSet(&info, 0, sizeof(info));
            CAM_Get_Frame_Info(CAM_USE_ID, &info);
        } while (info.validFrames == 0);
        picture = (uint8_t *)(uintptr_t)(info.curFrameAddr);
        length = info.curFrameBytes;
        CAM_Pop_Frame(CAM_USE_ID);
        pictureArr[idx] = picture;
        lengthArr[idx] = length;
        idx++;
        if (idx >= BUFF_SIZE / FRAME_SIZE) {
            for (uint32_t i=0; i<BUFF_SIZE / FRAME_SIZE; i++) {
                MSG("picture = 0x%08X, len = %d\r\n", pictureArr[i], lengthArr[i]);
            }
            MSG("all succeed!\r\n");
            MSG("dump binary memory 1.bin 0x%08X 0x%08X\r\n", buff, buff + BUFF_SIZE);
            for (uint32_t i=0; i<BUFF_SIZE; i++) {
                if (i % 16 == 0) {
                    MSG("\r\n");
                }
                MSG("%02X ", buff[i]);
            }
            CAM_DFE_Disable();
            CAM_Disable(CAM_USE_ID);
            while (1);
        }
    }

    BL_CASE_SUCCESS;
}

