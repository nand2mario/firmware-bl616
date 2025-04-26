#include "utils.h"

void fpga_tx_header(int cmd, int len) {
    bflb_uart_putchar(uart1_dev, 0xAA);
    bflb_uart_putchar(uart1_dev, len >> 8);
    bflb_uart_putchar(uart1_dev, len & 0xFF);
    bflb_uart_putchar(uart1_dev, cmd);
}

void fpga_tx_byte(uint8_t b) {
    bflb_uart_putchar(uart1_dev, b);
}
