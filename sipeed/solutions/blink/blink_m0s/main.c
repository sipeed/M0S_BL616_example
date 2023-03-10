#include "board.h"

#include "bflb_mtimer.h"
#include "bflb_gpio.h"

#define DBG_TAG "MAIN"
#include "log.h"

static uint8_t gpio_led_pins[] = {
    GPIO_PIN_0,
    GPIO_PIN_1,
    GPIO_PIN_2,
    GPIO_PIN_3,

    GPIO_PIN_10,
    GPIO_PIN_11,
    GPIO_PIN_12,
    GPIO_PIN_13,
    GPIO_PIN_14,
    GPIO_PIN_15,
    GPIO_PIN_16,
    GPIO_PIN_17,

    GPIO_PIN_20,
    // GPIO_PIN_21,
    // GPIO_PIN_22,

    GPIO_PIN_27,
    GPIO_PIN_28,
    GPIO_PIN_29,
    GPIO_PIN_30,
};

int main(void)
{
    board_init();
    struct bflb_device_s *gpio =
        bflb_device_get_by_name("gpio");

    for (size_t idx = 0; idx < sizeof(gpio_led_pins) / sizeof(gpio_led_pins[0]); idx++)
        bflb_gpio_init(gpio, gpio_led_pins[idx], GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);

    while (1) {
        LOG_I("gpio set\r\n");
        for (size_t idx = 0; idx < sizeof(gpio_led_pins) / sizeof(gpio_led_pins[0]); idx++)
            bflb_gpio_set(gpio, gpio_led_pins[idx]);
        bflb_mtimer_delay_ms(400);
        LOG_I("gpio reset\r\n");
        for (size_t idx = 0; idx < sizeof(gpio_led_pins) / sizeof(gpio_led_pins[0]); idx++)
            bflb_gpio_reset(gpio, gpio_led_pins[idx]);
        bflb_mtimer_delay_ms(600);
    }
}
