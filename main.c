#include <FreeRTOS.h>
#include "semphr.h"
#include "usbh_core.h"
#include "bflb_gpio.h"
#include "bflb_uart.h"
#include "board.h"
#include "bl616_glb.h"


extern void usbh_class_test(void);
extern void bflb_uart_set_console(struct bflb_device_s *dev);

int main(void)
{
    /* Board init (configures system clock, etc., and might init default UART0 console) */
    board_init();
    struct bflb_device_s *gpio_dev = bflb_device_get_by_name("gpio");
    /* Configure GPIO28 as UART TX and GPIO27 as UART RX */
    bflb_gpio_uart_init(gpio_dev, GPIO_PIN_28, GPIO_UART_FUNC_UART0_TX);    // JTAG connector pin 6
    bflb_gpio_uart_init(gpio_dev, GPIO_PIN_27, GPIO_UART_FUNC_UART0_RX);    // JTAG connector pin 7 (pin8 is GND, pin1 is VCC)
    /* Set up UART parameters */
    struct bflb_uart_config_s uart_cfg = {
        .baudrate = 115200,
        .data_bits = UART_DATA_BITS_8,
        .stop_bits = UART_STOP_BITS_1,
        .parity    = UART_PARITY_NONE,
        .tx_fifo_threshold = 7,
        .rx_fifo_threshold = 7,
        .flow_ctrl = 0,  /* No CTS/RTS flow control */
    };

    /* Get handle to UART0 */
    struct bflb_device_s *uart0_dev = bflb_device_get_by_name("uart0");
    /* Initialize UART1 with the config */
    bflb_uart_init(uart0_dev, &uart_cfg);

    /* Redirect standard I/O to UART1 */
    bflb_uart_set_console(uart0_dev);

    /* Now stdout goes to GPIO28, and stdin comes from GPIO27 */
    printf("UART console re-targeted to GPIO27/28 (UART1) at 115200 baud.\r\n");

    printf("Initializing USB host...\r\n");
    usbh_initialize(0, USB_BASE);
    usbh_class_test();
    vTaskStartScheduler();

    while (1) {
    }
}
