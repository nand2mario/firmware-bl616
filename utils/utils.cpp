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

uint32_t get_file_size(const char *fname) {
    FILINFO fno;
    FRESULT r = f_stat(fname, &fno);
    if (r != FR_OK) return 0;
    return fno.fsize;
}

// Send a romdata packet to core of len bytes in `fbuf`
void send_fbuf_data(uint16_t len) {
    taskENTER_CRITICAL();
    fpga_tx_header(0x07, len+1);
    for (int i = 0; i < len; i ++) {
        fpga_tx_byte(fbuf[i]);
    }
    taskEXIT_CRITICAL();
}

// set loading state
void set_loading_state(int state) {
    taskENTER_CRITICAL();
    fpga_tx_header(0x06, 2);
    fpga_tx_byte(state);        
    taskEXIT_CRITICAL();
}


// bring FPGA to a good state by sending a few 0's
void send_blank_packet(void) {
    taskENTER_CRITICAL();
    for (int i = 0; i < 8; i++) {
        fpga_tx_byte(0);
    }
    taskEXIT_CRITICAL();
}

#include <string>
#include <algorithm>

const char *cstr_find_ignore_case(const char *str, const char *substr) {
    std::string s(str);
    std::string ss(substr);
    auto it = std::search(s.begin(), s.end(), ss.begin(), ss.end(), [](char a, char b) {
        return std::tolower(a) == std::tolower(b);
    });
    if (it == s.end()) return NULL;
    return str + (it - s.begin());
}

/////////////////////////////////////////////////////////////////////////////////
// Shared state among tasks
volatile uint16_t joy1_state = 0;
volatile uint16_t joy2_state = 0;
volatile uint16_t hid1_state = 0;
volatile uint16_t hid2_state = 0;
volatile int16_t core_id = -1;
SemaphoreHandle_t state_mutex;              // for all global state access

// read joypad states
void get_joypad_states(uint16_t *joy1, uint16_t *joy2, uint16_t *hid1, uint16_t *hid2)
{
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
        *joy1 = joy1_state;
        *joy2 = joy2_state;
        *hid1 = hid1_state;
        *hid2 = hid2_state;
        xSemaphoreGive(state_mutex);
    }
}

// query over UART to return if the correct core is loaded
// return >= 0 if request is successful, -1 if timeout (200ms)
int16_t get_core_id(void) {
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
        core_id = -1;
        xSemaphoreGive(state_mutex);
    }

    // send command 1
    fpga_tx_header(0x01, 1);

    // TODO: use a queue for better performance
    uint64_t start = bflb_mtimer_get_time_ms();
    while (bflb_mtimer_get_time_ms() - start < 200) {
        if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
            int16_t res = core_id;
            if (res >= 0) {
                xSemaphoreGive(state_mutex);
                return res;
            }
            xSemaphoreGive(state_mutex);
        }
        delay(10);
    }
    return -1;
}