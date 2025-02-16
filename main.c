#include <FreeRTOS.h>
#include <stdarg.h>
#include "semphr.h"
#include "usbh_core.h"
#include "bflb_gpio.h"
#include "bflb_uart.h"
#include "board.h"
#include "bl616_glb.h"
#include "task.h"

extern void usbh_class_test(void);
extern void bflb_uart_set_console(struct bflb_device_s *dev);

// UART
struct bflb_device_s *gpio_dev;
struct bflb_device_s *uart0_dev;
struct bflb_device_s *uart1_dev;

// USB and fatfs
struct usbh_msc *msc;
// USB_NOCACHE_RAM_SECTION FATFS fs;
// USB_NOCACHE_RAM_SECTION FIL fnew;
// UINT fnum;
// FRESULT res_sd = 0;

// Add function prototype at the top
static void init_gpio_and_uart(void)
{
    gpio_dev = bflb_device_get_by_name("gpio");
    /* Core control UART */
    bflb_gpio_uart_init(gpio_dev, GPIO_PIN_28, GPIO_UART_FUNC_UART1_TX);    // JTAG connector pin 6
    bflb_gpio_uart_init(gpio_dev, GPIO_PIN_27, GPIO_UART_FUNC_UART1_RX);    // JTAG connector pin 7 (pin8 is GND, pin1 is VCC)
    /* Debug UART */
    bflb_gpio_uart_init(gpio_dev, GPIO_PIN_3, GPIO_UART_FUNC_UART0_TX);     // JTAG TDI
    bflb_gpio_uart_init(gpio_dev, GPIO_PIN_2, GPIO_UART_FUNC_UART0_RX);     // JTAG TDO

    /* Set up Core control UART parameters */
    struct bflb_uart_config_s uart1_cfg = {
        .baudrate = 2000000,    // 2Mbps
        .data_bits = UART_DATA_BITS_8,
        .stop_bits = UART_STOP_BITS_1,
        .parity    = UART_PARITY_NONE,
        .tx_fifo_threshold = 7,
        .rx_fifo_threshold = 7,
        .flow_ctrl = 0,  /* No CTS/RTS flow control */
    };
    /* Get handle to UART1 */
    uart1_dev = bflb_device_get_by_name("uart1");
    /* Initialize UART1 with the config */
    bflb_uart_init(uart1_dev, &uart1_cfg);
    /* Redirect standard I/O to UART1 */
    bflb_uart_set_console(uart1_dev);

    /* Set up Debug UART parameters */
    struct bflb_uart_config_s uart0_cfg = {
        .baudrate = 115200,    // 115200 baud
        .data_bits = UART_DATA_BITS_8,
        .stop_bits = UART_STOP_BITS_1,
        .parity    = UART_PARITY_NONE,
        .tx_fifo_threshold = 7,
        .rx_fifo_threshold = 7,
        .flow_ctrl = 0,  /* No CTS/RTS flow control */
    };
    /* Get handle to UART0 */
    uart0_dev = bflb_device_get_by_name("uart0");
    /* Initialize UART1 with the config */
    bflb_uart_init(uart0_dev, &uart0_cfg);
    /* Redirect standard I/O to UART1 */
    bflb_uart_set_console(uart0_dev);
}

bool uart0_active = false;
static uint32_t uart0_last_time;
// If uart0 is not used for 1 second, turn it off to restore JTAG operations
static void uart0_timeout(void) {
    uint32_t now = bflb_mtimer_get_time_ms();
    if (now - uart0_last_time > 1000) {
        bflb_gpio_deinit(gpio_dev, GPIO_PIN_3);
        bflb_gpio_deinit(gpio_dev, GPIO_PIN_2);
        uart0_active = false;
    }
}

static void uart0_on(void) {
    if (!uart0_active) {
        bflb_gpio_uart_init(gpio_dev, GPIO_PIN_3, GPIO_UART_FUNC_UART0_TX);     // JTAG TDI
        bflb_gpio_uart_init(gpio_dev, GPIO_PIN_2, GPIO_UART_FUNC_UART0_RX);     // JTAG TDO
        uart0_active = true;
    }
    uart0_last_time = bflb_mtimer_get_time_ms();
}

void overlay_cursor(int row, int col) {
    uart0_on();
    // uart1 command: 4 x[7:0] y[7:0]
    bflb_uart_putchar(uart1_dev, 0x04);       // command 4, move cursor
    bflb_uart_putchar(uart1_dev, col);
    bflb_uart_putchar(uart1_dev, row);
}

void overlay_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    char buf[256];
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    
    uart0_on();
    bflb_uart_putchar(uart1_dev, 0x05);       // command 5, display string
    for(int i = 0; buf[i] != '\0' && i < sizeof(buf); i++) {
        bflb_uart_putchar(uart1_dev, buf[i]);
    }
}

void overlay_clear() {
    overlay_cursor(0, 0);
    for (int i = 0; i < 28; i++) {
        overlay_printf("                                \r\n");
    }
}

static void main_task(void *pvParameters)
{
    bool redraw = true;

    while (1) {
        if (redraw) {
            overlay_clear();
            overlay_cursor(2, 10);
            //              01234567890123456789012345678901
            overlay_printf("=== Welcome to Tangcores ===\r\n");

            overlay_cursor(2, 12);
            overlay_printf("1) Load ROM from SD card\n");
            overlay_cursor(2, 13);
            overlay_printf("2) Select core\n");
            overlay_cursor(2, 14);
            overlay_printf("3) Options\n");
            // cursor(2, 15);
            // print("4) Verify core\n");
            overlay_cursor(2, 16);
            overlay_printf("Version: ");
            overlay_printf(__DATE__);
            redraw = false;
        }

        uart0_timeout();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


#define MAIN_TASK_STACK_SIZE  1024
#define MAIN_TASK_PRIORITY    2
static TaskHandle_t main_task_handle;

extern void fatfs_usbh_driver_register(void);

int main(void)
{
    /* Board init (configures system clock, etc., and might init default UART0 console) */
    board_init();
    
    // Initialize GPIO and UART
    init_gpio_and_uart();

    // Initializing USB host...
    usbh_initialize(0, USB_BASE);
    fatfs_usbh_driver_register();       // register the USB disk driver to FatFS
    // usbh_class_test();

    // Create the main task
    xTaskCreate(main_task, "main_task", MAIN_TASK_STACK_SIZE, NULL, MAIN_TASK_PRIORITY, &main_task_handle);
    vTaskStartScheduler();

    while (1) {
    }
}
