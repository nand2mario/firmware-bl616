extern "C" {
#include "bflb_gpio.h"
#include "board.h"
}

#include "utils.h"
#include "init.h"

extern "C" void bflb_uart_set_console(struct bflb_device_s *dev);

void init_gpio_and_uart() {
    // turn of UART0
    // uart0_dev = bflb_device_get_by_name("uart0");
    // bflb_uart_deinit(uart0_dev);

    gpio_dev = bflb_device_get_by_name("gpio");
    // deinit all GPIOs
    bflb_gpio_deinit(gpio_dev, GPIO_PIN_0);
    bflb_gpio_deinit(gpio_dev, GPIO_PIN_1);
    bflb_gpio_deinit(gpio_dev, GPIO_PIN_2);
    bflb_gpio_deinit(gpio_dev, GPIO_PIN_3);

    bflb_gpio_deinit(gpio_dev, GPIO_PIN_10);
    bflb_gpio_deinit(gpio_dev, GPIO_PIN_11);
    bflb_gpio_deinit(gpio_dev, GPIO_PIN_12);
    bflb_gpio_deinit(gpio_dev, GPIO_PIN_13);
    bflb_gpio_deinit(gpio_dev, GPIO_PIN_14);
    bflb_gpio_deinit(gpio_dev, GPIO_PIN_15);
    bflb_gpio_deinit(gpio_dev, GPIO_PIN_16);
    bflb_gpio_deinit(gpio_dev, GPIO_PIN_17);

    bflb_gpio_deinit(gpio_dev, GPIO_PIN_20);
    bflb_gpio_deinit(gpio_dev, GPIO_PIN_21);
    bflb_gpio_deinit(gpio_dev, GPIO_PIN_22);

    bflb_gpio_deinit(gpio_dev, GPIO_PIN_27);
    bflb_gpio_deinit(gpio_dev, GPIO_PIN_28);
    bflb_gpio_deinit(gpio_dev, GPIO_PIN_29);
    bflb_gpio_deinit(gpio_dev, GPIO_PIN_30);

    /* Core control UART 1 */
#ifdef TANG_PRIMER25K
    bflb_gpio_uart_init(gpio_dev, GPIO_PIN_11, GPIO_UART_FUNC_UART1_TX);    // JTAG connector pin 6
    bflb_gpio_uart_init(gpio_dev, GPIO_PIN_10, GPIO_UART_FUNC_UART1_RX);    // JTAG connector pin 7 (pin8 is GND, pin1 is VCC)
#elif defined(TANG_NANO20K)
    bflb_gpio_uart_init(gpio_dev, GPIO_PIN_11, GPIO_UART_FUNC_UART1_TX);    // JTAG connector pin 6
    bflb_gpio_uart_init(gpio_dev, GPIO_PIN_13, GPIO_UART_FUNC_UART1_RX);    // JTAG connector pin 7 (pin8 is GND, pin1 is VCC)
#else
    bflb_gpio_uart_init(gpio_dev, GPIO_PIN_28, GPIO_UART_FUNC_UART1_TX);    // JTAG connector pin 6
    bflb_gpio_uart_init(gpio_dev, GPIO_PIN_27, GPIO_UART_FUNC_UART1_RX);    // JTAG connector pin 7 (pin8 is GND, pin1 is VCC)
#endif

    /* Set up Core control UART parameters */
    struct bflb_uart_config_s uart1_cfg = {
        // .baudrate = 1000000,
#if defined(TANG_CONSOLE60K) || defined(TANG_CONSOLE138K)
        .baudrate = 2000000,
#else
        // all other boards have 26Mhz XTAL
        .baudrate = 2000000 * 40 / 26,
#endif
        .direction = UART_DIRECTION_TXRX,
        .data_bits = UART_DATA_BITS_8,
        .stop_bits = UART_STOP_BITS_1,
        .parity    = UART_PARITY_NONE,
        .bit_order = UART_LSB_FIRST,
        .flow_ctrl = 0,  /* No CTS/RTS flow control */
        .tx_fifo_threshold = 7,
        .rx_fifo_threshold = 7,
    };
    /* Get handle to UART1 */
    uart1_dev = bflb_device_get_by_name("uart1");
    /* Initialize UART1 with the config */
    bflb_uart_init(uart1_dev, &uart1_cfg);

    bflb_uart_set_console(uart1_dev);       // for debug

    // set JTAG pins to high-Z
    // interrupts masked, SWGPIO mode, output off, input off, schmitt ON
    const uint32_t GPIO_HIGH_Z = (1 << 22) | (0xB << 8) | (1 << 1);
    *reg_gpio_tms = GPIO_HIGH_Z;
    *reg_gpio_tck = GPIO_HIGH_Z;
    *reg_gpio_tdo = GPIO_HIGH_Z;
    *reg_gpio_tdi = GPIO_HIGH_Z;

    // Initialize SD pins
    board_sdh_gpio_init();
    // bflb_gpio_init(gpio, GPIO_PIN_10, GPIO_FUNC_SDH | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_2);  // D1
    // bflb_gpio_init(gpio, GPIO_PIN_11, GPIO_FUNC_SDH | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_2);  // D0
    // bflb_gpio_init(gpio, GPIO_PIN_12, GPIO_FUNC_SDH | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_2);  // CLK
    // bflb_gpio_init(gpio, GPIO_PIN_13, GPIO_FUNC_SDH | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_2);  // CMD
    // bflb_gpio_init(gpio, GPIO_PIN_14, GPIO_FUNC_SDH | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_2);  // D3
    // bflb_gpio_init(gpio, GPIO_PIN_15, GPIO_FUNC_SDH | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_2);  // D2

    // Set GPIO 1 (physical pin 15) to high to enable SDMMC
    bflb_gpio_init(gpio_dev, GPIO_PIN_16, GPIO_OUTPUT | GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_3);
    bflb_gpio_set(gpio_dev, GPIO_PIN_16);
}
