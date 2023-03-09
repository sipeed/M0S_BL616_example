#include <assert.h>
#include <stdint.h>

#include "board.h"

#define DBG_TAG "MAIN"
#include "log.h"

#include "usbd_core.h"
#include "bflb_gpio.h"
#include "bflb_uart.h"
#include "ring_buffer.h"
#include "csi_core.h"

#include "cfg.h"

Ring_Buffer_Type uart1_rx_rb;

static struct bflb_device_s *gpio;
static struct bflb_device_s *uart1;

static void uart1_gpio_init(void);
static void uart1_ringbuffer_init(void);
static void uart1_init(void);

int main(void)
{
    board_init();
    uart1_gpio_init();
    {
        static uint8_t uartx_rx_buffer[2 * 512];
        Ring_Buffer_Init(&uart1_rx_rb, uartx_rx_buffer, sizeof(uartx_rx_buffer), NULL, NULL);
    }
    uart1_init();

    extern void usbd_cdc_acm_template_init(void);
    usbd_cdc_acm_template_init();

    LOG_D("start loop\r\n");
    for (size_t loop_count = 0;;) {
        extern volatile bool ep_tx_busy_flag;
        if (ep_tx_busy_flag)
            continue;

        size_t uart1_rx_rb_len = Ring_Buffer_Get_Length(&uart1_rx_rb);
        if (!uart1_rx_rb_len)
            continue;

        if (uart1_rx_rb_len < 512 && loop_count++ < 1000) {
            continue;
        }
        loop_count = 0;

        uint8_t data[uart1_rx_rb_len];
        size_t uart1_rx_rb_len_acc = Ring_Buffer_Read(&uart1_rx_rb, data, uart1_rx_rb_len);
        if (!uart1_rx_rb_len_acc)
            continue;

        LOG_D("acc: %u, uart1_rx_rb_len: %u\r\n", uart1_rx_rb_len_acc, uart1_rx_rb_len);
        for (size_t i = 0; i < uart1_rx_rb_len_acc; i++) {
            LOG_RD("%c", data[i]);
        }
        LOG_RD("\r\n");

        csi_dcache_clean_invalid_range(data, uart1_rx_rb_len_acc);
        ep_tx_busy_flag = true;
        usbd_ep_start_write(CDC_IN_EP, data, uart1_rx_rb_len_acc);
    }
}

void usbd_cdc_acm_bulk_out_cb(uint8_t *buf, size_t len, uint8_t ep)
{
    assert(ep == CDC_OUT_EP);
    // usbd_ep_start_write(CDC_IN_EP, buf, len);
    bflb_uart_txint_mask(uart1, false);
    for (size_t i = 0; i < len; i++) {
        bflb_uart_putchar(uart1, buf[i]);
    }
}

static void uart1_gpio_init(void)
{
    gpio = bflb_device_get_by_name("gpio");
    bflb_gpio_uart_init(gpio, GPIO_PIN_UART1_TX, GPIO_UART_FUNC_UART1_TX);
    bflb_gpio_uart_init(gpio, GPIO_PIN_UART1_RX, GPIO_UART_FUNC_UART1_RX);
}

static void uart_isr(int irq, void *arg)
{
    uint32_t intstatus = bflb_uart_get_intstatus(uart1);

    if (intstatus & UART_INTSTS_RX_FIFO) {
        LOG_D("rx fifo: ");
        while (bflb_uart_rxavailable(uart1)) {
            char c = bflb_uart_getchar(uart1);
            LOG_RT("0x%02x\r\n", c);
            Ring_Buffer_Write_Byte(&uart1_rx_rb, c);
        }
        LOG_RD("\r\n");
        bflb_uart_feature_control(uart1, UART_CMD_SET_RTS_VALUE, 1);
    }
    if (intstatus & UART_INTSTS_RTO) {
        LOG_D("rto: ");
        while (bflb_uart_rxavailable(uart1)) {
            char c = bflb_uart_getchar(uart1);
            LOG_RT("%02x ", c);
            Ring_Buffer_Write_Byte(&uart1_rx_rb, c);
        }
        LOG_RD("\r\n");
        bflb_uart_int_clear(uart1, UART_INTCLR_RTO);
    }
    if (intstatus & UART_INTSTS_TX_FIFO) {
        LOG_D("tx fifo\r\n");
        bflb_uart_txint_mask(uart1, true);
    }
}

static void uart1_init(void)
{
    uart1 = bflb_device_get_by_name("uart1");

    struct bflb_uart_config_s cfg;
    cfg.baudrate = 2000000;
    cfg.data_bits = UART_DATA_BITS_8;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.parity = UART_PARITY_NONE;
    cfg.flow_ctrl = 0;
    cfg.tx_fifo_threshold = 7;
    cfg.rx_fifo_threshold = 0;
    bflb_uart_init(uart1, &cfg);

    bflb_irq_attach(uart1->irq_num, uart_isr, NULL);
    bflb_irq_enable(uart1->irq_num);
    bflb_uart_rxint_mask(uart1, false);
}
