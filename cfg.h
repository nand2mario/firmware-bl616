#pragma once

// nand2mario: this is similar to sipheed_solutions/bl616_fpga_partner/exported/cfg.h

#include "bflb_core.h"
extern struct bflb_device_s *gpio_hdl;
extern struct bflb_device_s *uart_hdl;

static inline void gpio_init(void)
{
    struct bflb_device_s *gpio = gpio_hdl;

    // deinit all GPIOs
    bflb_gpio_deinit(gpio, GPIO_PIN_0);
    bflb_gpio_deinit(gpio, GPIO_PIN_1);
    bflb_gpio_deinit(gpio, GPIO_PIN_2);
    bflb_gpio_deinit(gpio, GPIO_PIN_3);

    bflb_gpio_deinit(gpio, GPIO_PIN_10);
    bflb_gpio_deinit(gpio, GPIO_PIN_11);
    bflb_gpio_deinit(gpio, GPIO_PIN_12);
    bflb_gpio_deinit(gpio, GPIO_PIN_13);
    bflb_gpio_deinit(gpio, GPIO_PIN_14);
    bflb_gpio_deinit(gpio, GPIO_PIN_15);
    bflb_gpio_deinit(gpio, GPIO_PIN_16);
    bflb_gpio_deinit(gpio, GPIO_PIN_17);

    bflb_gpio_deinit(gpio, GPIO_PIN_20);
    bflb_gpio_deinit(gpio, GPIO_PIN_21);
    bflb_gpio_deinit(gpio, GPIO_PIN_22);

    bflb_gpio_deinit(gpio, GPIO_PIN_27);
    bflb_gpio_deinit(gpio, GPIO_PIN_28);
    bflb_gpio_deinit(gpio, GPIO_PIN_29);
    bflb_gpio_deinit(gpio, GPIO_PIN_30);

    // UART1 on GPIO TX on GPIO 27 (SOM jtag connector pin 6), RX on GPIO 28 (SOM jtag connector pin 7)
    bflb_gpio_uart_init(gpio, GPIO_PIN_28, GPIO_UART_FUNC_UART1_TX);    // IOB91B, BANK8_V14, JTAG connector pin 6
    bflb_gpio_uart_init(gpio, GPIO_PIN_27, GPIO_UART_FUNC_UART1_RX);    // JTAG connector pin 7 (pin 8 is GND, pin 1 is VCC)

}
