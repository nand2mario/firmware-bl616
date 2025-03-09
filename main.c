/*
 * TangCore firmware for BL616 MCU
 *
 * (c) 2025, nand2mario <nand2mario@outlook.com>
 *
 * This source code is licensed under the Apache 2.0 license found in the
 * LICENSE file in the root directory of this source tree. 
 *
 */

#include <stdarg.h>
#include <string.h>

#include "board.h"
#include "bl616_glb.h"
#include "bflb_gpio.h"
#include "bflb_uart.h"
#include "bflb_clock.h"
#include "bl616_clock.h"

#include "usbh_core.h"
#include "ff.h"

#include "programmer.h"
#include "usb_gamepad.h"
#include "utils.h"

/////////////////////////////////////////////////////////////////////////////////
// Global state

int option_osd_key = OPTION_OSD_KEY_SELECT_RIGHT;
int16_t active_core = -1;           // firmware detected this core as active
bool core_running;                  // a rom is loaded and running on the core

// UART
struct bflb_device_s *gpio_dev;
struct bflb_device_s *uart0_dev;
struct bflb_device_s *uart1_dev;

// USB and fatfs
struct usbh_msc *msc;
char fname[1024];

// Tasks and shared state
TaskHandle_t main_task_handle;
TaskHandle_t uart1_rx_task_handle;
volatile uint16_t joy1_state = 0;
volatile uint16_t joy2_state = 0;
volatile int16_t core_id = -1;
volatile uint16_t hid1_state = 0;
volatile uint16_t hid2_state = 0;
SemaphoreHandle_t state_mutex;              // for all global state access

// Core specific state
bool gba_bios_loaded;
bool gba_missing_bios_warned;

#ifdef TANG_CONSOLE60K
const char *BOARD_NAME = "console60k";
#elif defined(TANG_CONSOLE138K)
const char *BOARD_NAME = "console138k";
#elif defined(TANG_MEGA60K)
const char *BOARD_NAME = "mega60k";
#elif defined(TANG_MEGA138K)
const char *BOARD_NAME = "mega138k";
#elif defined(TANG_PRIMER25K)
const char *BOARD_NAME = "primer25k";
#elif defined(TANG_NANO20K)
const char *BOARD_NAME = "nano20k";
#else
const char *BOARD_NAME = "unknown";
#endif

/////////////////////////////////////////////////////////////////////////////////
// GPIO and UART

static void init_gpio_and_uart(void)
{
    // turn of UART0
    uart0_dev = bflb_device_get_by_name("uart0");
    bflb_uart_deinit(uart0_dev);

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
    bflb_uart_set_console(uart1_dev);       // for debug

    // set JTAG pins to high-Z
    // interrupts masked, SWGPIO mode, output off, input off, schmitt ON
    const uint32_t GPIO_HIGH_Z = (1 << 22) | (0xB << 8) | (1 << 1);
    *reg_gpio_tms = GPIO_HIGH_Z;
    *reg_gpio_tck = GPIO_HIGH_Z;
    *reg_gpio_tdo = GPIO_HIGH_Z;
    *reg_gpio_tdi = GPIO_HIGH_Z;
}

void enable_jtag_pins(void) {
    // JTAG pins
    bflb_gpio_init(gpio_dev, GPIO_PIN_JTAG_TMS, GPIO_OUTPUT | GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_3);
    bflb_gpio_init(gpio_dev, GPIO_PIN_JTAG_TCK, GPIO_OUTPUT | GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_3);
    bflb_gpio_init(gpio_dev, GPIO_PIN_JTAG_TDI, GPIO_OUTPUT | GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_3);
    bflb_gpio_init(gpio_dev, GPIO_PIN_JTAG_TDO, GPIO_INPUT  | GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_3);
}

void disable_jtag_pins(void) {
    bflb_gpio_deinit(gpio_dev, GPIO_PIN_JTAG_TMS);
    bflb_gpio_deinit(gpio_dev, GPIO_PIN_JTAG_TCK);
    bflb_gpio_deinit(gpio_dev, GPIO_PIN_JTAG_TDI);
    bflb_gpio_deinit(gpio_dev, GPIO_PIN_JTAG_TDO);
}


/////////////////////////////////////////////////////////////////////////////////
// Overlay and other core control over UART

int _overlay_on = 1;

int overlay_on() {
    return _overlay_on;
}

void overlay_cursor(int col, int row) {
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
    
    bflb_uart_putchar(uart1_dev, 0x05);       // command 5, display string
    for(int i = 0; buf[i] != '\0' && i < sizeof(buf); i++) {
        bflb_uart_putchar(uart1_dev, buf[i]);
    }
    bflb_uart_putchar(uart1_dev, '\0');
}

void overlay_clear() {
    for (int i = 0; i < 28; i++) {
        overlay_cursor(0, i);
        //              01234567890123456789012345678901
        overlay_printf("                                ");
    }
}

void overlay_status(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    char buf[256];
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    overlay_cursor(1, 27);
    overlay_printf(buf);
}

// show a pop-up message, press any key to discard (caller needs to redraw screen)
// msg: could be multi-line (separate with \n), max 10 lines
// center: whether to center the text
void overlay_message(char *msg, int center) {
    // count number of lines and max width
    int w[10], lines=10, maxw = 0;
    int len = strlen(msg);
    char *end = msg + len;
    char *sol = msg;
    for (int i = 0; i < 10; i++) {
        char *eol = strchr(sol, '\n');
        if (eol) { // found \n
            w[i] = min(eol - sol, 26);
            maxw = max(w[i], maxw);
            sol = eol+1;
        } else {
            w[i] = min(end - sol, 26);
            maxw = max(w[i], maxw);
            lines = i+1;
            break;
        }		
    }
    // status("");
    // printf("w=%d, lines=%d", maxw, lines);
    // draw a box 
    int y0 = 14 - ((lines + 2) >> 1);
    int y1 = y0 + lines + 2;
    int x0 = 16 - ((maxw + 2) >> 1);
    int x1 = x0 + maxw + 2;
    for (int y = y0; y < y1; y++)
        for (int x = x0; x < x1; x++) {
            overlay_cursor(x, y);
            if ((x == x0 || x == x1-1) && (y == y0 || y == y1-1))
                overlay_printf("+");
            else if (x == x0 || x == x1-1)
                overlay_printf("|");
            else if (y == y0 || y == y1-1)
                overlay_printf("-");
            else
                overlay_printf(" ");
        }
    // print text
    char *s = msg;
    for (int i = 0; i < lines; i++) {
        if (center)
            overlay_cursor(16-(w[i]>>1), y0+i+1);
        else
            overlay_cursor(x0+1, y0+i+1);
        while (*s != '\n' && *s != '\0') {
            overlay_printf("%c", *s);
            s++;
        }
        s++;
    }
    // wait for a keypress
    delay(300);
    for (;;) {
        uint16_t joy1=0, joy2=0, hid1=0, hid2=0;
        get_joypad_states(&joy1, &joy2, &hid1, &hid2);
        joy1 |= hid1; joy2 |= hid2;
        if ((joy1 & 0x1) || (joy1 & 0x100) || (joy2 & 0x1) || (joy2 & 0x100))
            break;
    }
    delay(300);
}

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

    bflb_uart_putchar(uart1_dev, 0x01);
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

// set loading state
void set_loading_state(int state) {
    bflb_uart_putchar(uart1_dev, 6);        // 6 loadingstate[7:0]
    bflb_uart_putchar(uart1_dev, state);        
}

// turn overlay on/off
void overlay(int state) {
    _overlay_on = state;
    bflb_uart_putchar(uart1_dev, 8);        // 8 x[7:0]
    bflb_uart_putchar(uart1_dev, state);        
}

// bring FPGA to a good state by sending a few 0's
void send_blank_packet(void) {
    for (int i = 0; i < 8; i++) {
        bflb_uart_putchar(uart1_dev, 0);
    }
}

/////////////////////////////////////////////////////////////////////////////////
// Core loading and other file system operations

// nand2mario: these USB data structures cannot be UNCACHED as they are written to by hardware
USB_NOCACHE_RAM_SECTION FATFS fs;
USB_NOCACHE_RAM_SECTION FIL fcore;
#define BLOCK_SIZE (8*1024)
static USB_NOCACHE_RAM_SECTION BYTE __attribute__((aligned(64))) fbuf[BLOCK_SIZE];

FRESULT res_sd = 0;
#define PAGESIZE 22
#define TOPLINE 2
#define PWD_SIZE 1024
char pwd[PWD_SIZE];
// one page of file names to display
char file_names[PAGESIZE][256];
int file_dir[PAGESIZE];         // this file is a directory
int file_sizes[PAGESIZE];       
int file_len;		            // number of files on this page

uint32_t get_file_size(const char *fname) {
    FILINFO fno;
    FRESULT r = f_stat(fname, &fno);
    if (r != FR_OK) return 0;
    return fno.fsize;
}

bool load_core(const char *fname) {
    FRESULT res_sd = f_mount(&fs, "usb:", 1);
    if (res_sd != FR_OK) {
        overlay_printf("mount fail, res:%d\r\n", res_sd);
        return false;
    }
    int len = get_file_size(fname);
    overlay_status("Writing %u bytes...", len);

    res_sd = f_open(&fcore, fname, FA_READ);
    if (res_sd != FR_OK) {
        overlay_printf("open fail, res:%d\r\n", res_sd);
        return false;
    }
    bool res = false;

    chain_len = detectChain(JTAG_MAX_CHAIN);
    if (chain_len == 0 || (    idcodes[0] != IDCODE_GW5AT_60 
                            && idcodes[0] != IDCODE_GWAST_138
                            && idcodes[0] != IDCODE_GW5A_25
                            && idcodes[0] != IDCODE_GW2A_18)) {
        overlay_printf("No known board detected, IDCODE=%08x\n", idcodes[0]);
        goto load_core_close;
    }

    if (!eraseSRAM()) {
        overlay_printf("Failed to erase SRAM\n");
        goto load_core_close;
    }

    overlay_status("Erasing again...");
    if (!eraseSRAM()) {
        overlay_printf("Failed to erase SRAM 2nd time\n");
        goto load_core_close;
    }    

    if (!writeSRAM_start()) {
        overlay_printf("Failed to start write SRAM\n");
        goto load_core_close;
    }

    BYTE *fbuf_cached;
    fbuf_cached = malloc(BLOCK_SIZE);
    if (!fbuf_cached) {
        overlay_printf("Cannot malloc buffer\r\n");
        goto load_core_close;
    }

#define JTAG_FAST

    UINT bytes = 0, total = 0;
    uint64_t time_total = bflb_mtimer_get_time_us();
    uint64_t time_jtag = 0, time_flash = 0;
    extern uint64_t jtag_writetdi_time;
    uint64_t writetdi_time_start = jtag_writetdi_time;
#ifdef JTAG_FAST
    taskENTER_CRITICAL();
    jtag_enter_gpio_out_mode();
    for (;;) {
        f_read(&fcore, fbuf, BLOCK_SIZE, &bytes);
        if (bytes == 0) break;
        total += bytes;
        jtag_writeTDI_msb_first_gpio_out_mode(fbuf, bytes, total >= len);
        if (bytes < BLOCK_SIZE) break;
    }
    jtag_exit_gpio_out_mode();
    if (!writeSRAM_end()) {
        overlay_status("Failed to program SRAM\n");
        goto load_core_close;
    }
    taskEXIT_CRITICAL();

#else
    taskENTER_CRITICAL();
    for (;;) {
        uint64_t time_flash_start = bflb_mtimer_get_time_us();
        f_read(&fcore, fbuf, BLOCK_SIZE, &bytes);
        time_flash += bflb_mtimer_get_time_us() - time_flash_start;
        // overlay_status("f_read: offset=%u, bytes=%d, 4 bytes=%02x %02x %02x %02x", 
        //     (uint32_t)f_tell(&fcore), bytes, fbuf[0], fbuf[1], fbuf[2], fbuf[3]);

        if (bytes == 0) break;
        // reverse msb/lsb as Gowin bitstream needs to MSB first
        // also copy to cached memory for better performance
        static unsigned char lookup[16] = {
            0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
            0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf, };
        for (int j = 0; j < bytes; j++)
            fbuf_cached[j] = lookup[fbuf[j] & 0xf] << 4 | lookup[fbuf[j] >> 4];

        total += bytes;
        uint64_t time_jtag_start = bflb_mtimer_get_time_us();
        if (!writeSRAM_send(fbuf_cached, bytes*8, total >= len)) {
            overlay_status("Failed to send data to SRAM\n");
            goto free_fbuf_cached;
        }
        time_jtag += bflb_mtimer_get_time_us() - time_jtag_start;
        if (bytes < BLOCK_SIZE) break;
    } 
    if (!writeSRAM_end()) {
        overlay_status("Failed to program SRAM\n");
        goto free_fbuf_cached;
    }
    taskEXIT_CRITICAL();

    free(fbuf_cached);
#endif

    time_total = bflb_mtimer_get_time_us() - time_total;
    overlay_status("Time: total=%lld us, jtag=%lld us, flash=%lld us, writetdi=%lld us", time_total, time_jtag, 
        time_flash, jtag_writetdi_time - writetdi_time_start);

    // printf("Status after program sram: %x\n", readStatusReg());
    res = true;

load_core_close:
    f_close(&fcore);
    return res;
}

// starting from `start`, load `len` file names into file_names, 
// file_dir. 
// `*count` is set to number of all valid entries and `file_len` is
// set to valid entries on this page.
// `filter` when non-null is a function that takes a file name and returns true if the file is
// to be included in the list.
// return: 0 if successful
int load_dir(char *dir, int start, int len, int *count, bool (*filter)(char *)) {
    // DEBUG("load_dir: %s, start=%d, len=%d\n", dir, start, len);
    int cnt = 0;
    DIR d;
    file_len = 0;

    if (f_opendir(&d, dir) != 0) {
        return -1;
    }
    // an entry to return to parent dir or main menu 
    int is_root = dir[1] == '\0';
    if (start == 0 && len > 0) {
        if (is_root) {
            strncpy(file_names[0], "<< Return to main menu", 256);
            file_dir[0] = 0;
        } else {
            strncpy(file_names[0], "..", 256);
            file_dir[0] = 1;
        }
        file_len++;
    }
    cnt++;

    // generate all file entries
    FILINFO fno;
    while (f_readdir(&d, &fno) == FR_OK) {
        if (fno.fname[0] == 0)
            break;
        if ((fno.fattrib & AM_HID) || (fno.fattrib & AM_SYS))
             // skip hidden and system files
            continue;
        if (filter && !filter(fno.fname))
            continue;
        if (cnt >= start && file_len < len) {
            strncpy(file_names[file_len], fno.fname, 256);
            file_dir[file_len] = fno.fattrib & AM_DIR;
            file_sizes[file_len] = fno.fsize;
            file_len++;
//            DEBUG("%s\n", fno.fname);
        }
        cnt++;
    }
    f_closedir(&d);
    *count = cnt;
    // DEBUG("load_dir: count=%d\n", cnt);
    return 0;
}

// Send a romdata packet to core of len bytes in `fbuf`
void send_fbuf_data(int len) {
    bflb_uart_putchar(uart1_dev, 7);        // 7 len[23:0] <data>
    bflb_uart_putchar(uart1_dev, (len >> 16) & 0xff);  // MSB first
    bflb_uart_putchar(uart1_dev, (len >> 8) & 0xff);
    bflb_uart_putchar(uart1_dev, len & 0xff);
    for (int i = 0; i < len; i ++) {
        bflb_uart_putchar(uart1_dev, fbuf[i]);
    }
}

// Load a NES ROM
// return 0 if successful
int loadnes(const char *fname) {
    int r = 1;
    DEBUG("loadnes start\n");

    // check extension .nes
    char *p = strcasestr(fname, ".nes");
    if (p == NULL) {
        overlay_status("Only .nes supported");
        goto loadnes_end;
    }

    r = f_open(&fcore, fname, FA_READ);
    if (r) {
        overlay_status("Cannot open file");
        goto loadnes_end;
    }
    unsigned int off = 0, br, total = 0;
    unsigned int size = get_file_size(fname);

    // load actual ROM
    set_loading_state(1);
    core_running = false;

    // Send rom content
    if ((r = f_lseek(&fcore, off)) != FR_OK) {
        overlay_status("Seek failure");
        goto loadnes_snes_end;
    }


    do {
        if ((r = f_read(&fcore, fbuf, BLOCK_SIZE, &br)) != FR_OK)
            break;
        // start rom loading command
        send_fbuf_data(br);
        total += br;
        if ((total & 0xfff) == 0) {	// display progress every 4KB
            //              01234567890123456789012345678901
            overlay_status("%d/%dK                          ", total >> 10, size >> 10);
        }
    } while (br == BLOCK_SIZE);

    DEBUG("loadnes: %d bytes\n", total);
    overlay_status("Success");
    core_running = true;

    overlay(0);		// turn off OSD

loadnes_snes_end:
    set_loading_state(0);   // turn off game loading, this starts the core
    f_close(&fcore);
loadnes_end:
    return r;
}

// return 0 if snes header is successfully parsed at off
// typ 0: LoROM, 1: HiROM, 2: ExHiROM
int parse_snes_header(FIL *fp, int pos, int file_size, int typ, unsigned char *hdr,
                      int *map_ctrl, int *rom_type_header, int *rom_size,
                      int *ram_size, int *company) {
    unsigned int br;
    if (f_lseek(fp, pos))
        return 1;
    f_read(fp, hdr, 64, &br);
    if (br != 64) return 1;
    int mc = hdr[21];
    int rom = hdr[23];
    int ram = hdr[24];
    int checksum = (hdr[28] << 8) + hdr[29];
    int checksum_compliment = (hdr[30] << 8) + hdr[31];
    int reset = (hdr[61] << 8) + hdr[60];
    int size2 = 1024 << rom;

    overlay_status("size=%d", size2);

    // calc heuristics score
    int score = 0;		
    if (size2 >= file_size) score++;
    if (rom == 1) score++;
    if (checksum + checksum_compliment == 0xffff) score++;
    int all_ascii = 1;
    for (int i = 0; i < 21; i++)
        if (hdr[i] < 32 || hdr[i] > 127)
            all_ascii = 0;
    score += all_ascii;

    overlay_status("pos=%x, type=%d, map_ctrl=%d, rom=%d, ram=%d, checksum=%x, checksum_comp=%x, reset=%x, score=%d\n", 
            pos, typ, mc, rom, ram, checksum, checksum_compliment, reset, score);

    if (rom < 14 && ram <= 7 && score >= 1 && 
        reset >= 0x8000 &&				// reset vector position correct
       ((typ == 0 && (mc & 3) == 0) || 	// normal LoROM
        (typ == 0 && mc == 0x53)    ||	// contra 3 has 0x53 and LoROM
        (typ == 1 && (mc & 3) == 1) ||	// HiROM
        (typ == 2 && (mc & 3) == 2))) {	// ExHiROM
        *map_ctrl = mc;
        *rom_type_header = hdr[22];
        *rom_size = rom;
        *ram_size = ram;
        *company = hdr[26];
        return 0;
    }
    return 1;
}

// TODO: implement bsram backup
// return 0 if successful
int loadsnes(const char *fname) {
    int r = 1;
    DEBUG("loadsnes start");

    // check extension .sfc or .smc
    char *p = strcasestr(fname, ".sfc");
    if (p == NULL)
        p = strcasestr(fname, ".smc");
    if (p == NULL) {
        overlay_status("Only .smc or .sfc supported");
        goto loadsnes_end;
    }

    r = f_open(&fcore, fname, FA_READ);
    if (r) {
        overlay_status("Cannot open file");
        goto loadsnes_end;
    }
    unsigned int br, total = 0;
    int size = get_file_size(fname);
    int map_ctrl, rom_type_header, rom_size, ram_size, company;
    // parse SNES header from ROM file
    int off = size & 0x3ff;		// rom header (0 or 512)
    int header_pos;
    overlay_status("snes rom header offset: %d\n", off);
    
    header_pos = 0x7fc0 + off;
    if (parse_snes_header(&fcore, header_pos, size-off, 0, fbuf, &map_ctrl, &rom_type_header, &rom_size, &ram_size, &company)) {
        header_pos = 0xffc0 + off;
        if (parse_snes_header(&fcore, header_pos, size-off, 1, fbuf, &map_ctrl, &rom_type_header, &rom_size, &ram_size, &company)) {
            header_pos = 0x40ffc0 + off;
            if (parse_snes_header(&fcore, header_pos, size-off, 2, fbuf, &map_ctrl, &rom_type_header, &rom_size, &ram_size, &company)) {
                overlay_status("Not a SNES ROM file");
                delay(200);
                goto loadsnes_close_file;
            }
        }
    }

    // load actual ROM
    set_loading_state(1);		// enable game loading, this resets SNES
    core_running = false;

    // Send 64-byte header to snes
    send_fbuf_data(64);

    // Send rom content to snes
    if ((r = f_lseek(&fcore, off)) != FR_OK) {
        overlay_status("Seek failure");
        goto loadsnes_snes_end;
    }
    do {
        if ((r = f_read(&fcore, fbuf, BLOCK_SIZE, &br)) != FR_OK)
            break;
        if (br == 0) break;
        send_fbuf_data(br);
        total += br;
        if ((total & 0xffff) == 0) {	// display progress every 64KB
            overlay_status("%d/%dK", total >> 10, size >> 10);
            if ((map_ctrl & 3) == 0)
                overlay_printf(" Lo");
            else if ((map_ctrl & 3) == 1)
                overlay_printf(" Hi");
            else if ((map_ctrl & 3) == 2)
                overlay_printf(" ExHi");
            //              01234567890123456789012345678901
            overlay_printf(" ROM=%d RAM=%d                 ", 1 << rom_size, ram_size ? (1 << ram_size) : 0);
        }
    } while (br == BLOCK_SIZE);

    overlay_status("Success");
    core_running = true;

    overlay(0);		// turn off OSD

loadsnes_snes_end:
    set_loading_state(0);	// turn off game loading, this starts SNES
loadsnes_close_file:
    f_close(&fcore);
loadsnes_end:
    return r;
}

// check if gba_bios.bin is present in the root directory
// if not, warn user, if present, load it
void gba_load_bios() {
    if (gba_bios_loaded | gba_missing_bios_warned) return;

    DEBUG("gba_load_bios start\n");
    FILINFO fno;
    if (f_stat("usb:gba/gba_bios.bin", &fno) != FR_OK) {
        overlay_message( "Cannot find /gba_bios.bin\n"
                 "Using open source BIOS\n"
                 "Expect low compatibility", 1);
        gba_missing_bios_warned = 1;
        return;
    }

    int r = 1;
    unsigned br;
    if (f_open(&fcore, "usb:gba/gba_bios.bin", FA_READ) != FR_OK) {
        overlay_message("Cannot open /gba/gba_bios.bin", 1);
        return;
    }
    set_loading_state(4);
    do {
        if ((r = f_read(&fcore, fbuf, 1024, &br)) != FR_OK)
            break;
        send_fbuf_data(br);
    } while (br == 1024);

    f_close(&fcore);
    gba_bios_loaded = 1;
    DEBUG("gba_load_bios end\n");
}

int loadgba(const char *fname) {
    DEBUG("loadgba start\n");
    FRESULT r = -1;

    // check extension .gba
    char *p = strcasestr(fname, ".gba");
    if (p == NULL) {
        overlay_status("Only .gba supported");
        goto loadgba_end;
    }

    unsigned int size = get_file_size(fname);

    r = f_open(&fcore, fname, FA_READ);
    if (r) {
        overlay_status("Cannot open file");
        goto loadgba_end;
    }
    unsigned int off = 0, br, total = 0;

    // load actual ROM
    set_loading_state(1);		// enable game loading, this resets GBA
    core_running = false;

    // Send rom content to gba
    if ((r = f_lseek(&fcore, off)) != FR_OK) {
        overlay_status("Seek failure");
        goto loadgba_close;
    }
    // int detect = 0; // 1: past 'EEPR', 2: past 'FLAS', 3: past 'SRAM'
    // gba_backup_type = GBA_BACKUP_NONE;
    do {
        if ((r = f_read(&fcore, fbuf, 1024, &br)) != FR_OK)
            break;

        send_fbuf_data(br);
        // TODO: do backup type detection

        total += br;
        if ((total & 0xffff) == 0) {	// display progress every 64KB
            //              01234567890123456789012345678901
            overlay_status("%d/%dK                          ", total >> 10, size >> 10);
        }
    } while (br == 1024);

    DEBUG("loadgba: %d bytes rom sent.\n", total); 

    gba_load_bios();

    overlay_status("Success");
    core_running = true;

    overlay(0);		// turn off OSD

loadgba_close:
    set_loading_state(0);   // turn off game loading, this starts the core
    f_close(&fcore);
loadgba_end:
    return r;
}

int loadmd(const char *fname) {
    DEBUG("loadmd start\n");
    FRESULT r = -1;

    // check extension .bin
    char *p = strcasestr(fname, ".bin");
    if (p == NULL) {
        overlay_status("Only .bin supported");
        goto loadmd_end;
    }

    r = f_open(&fcore, fname, FA_READ);
    if (r) {
        overlay_status("Cannot open file");
        goto loadmd_end;
    }
    unsigned int off = 0, br, total = 0;
    unsigned int size = get_file_size(fname);

    // load actual ROM
    set_loading_state(1);		// enable game loading, this resets the core
    core_running = false;

    // Send rom content to core
    if ((r = f_lseek(&fcore, off)) != FR_OK) {
        overlay_status("Seek failure");
        goto loadmd_close_file;
    }
    do {
        if ((r = f_read(&fcore, fbuf, 1024, &br)) != FR_OK)
            break;
        send_fbuf_data(br);
        total += br;
        if ((total & 0xfff) == 0) {	// display progress every 4KB
            //              01234567890123456789012345678901
            overlay_status("%d/%dK                          ", total >> 10, size >> 10);
        }
    } while (br == 1024);

    DEBUG("loadmd: %d bytes\n", total);
    overlay_status("Success");
    core_running = true;

    overlay(0);		// turn off OSD

loadmd_close_file:
    set_loading_state(0);   // turn off game loading, this starts the core
    f_close(&fcore);
loadmd_end:
    return r;
}


/////////////////////////////////////////////////////////////////////////////////
// Menu display and user interaction

// Menus for "NES", "SNES" ... entries
// dir: initial dir
// return 0: user chose a ROM (*choice), 1: no choice made, -1: error
// file chosen: pwd / file_name[*choice]
static int menu_loadrom(const char *dir) {
    res_sd = f_mount(&fs, "usb:", 1);
    if (res_sd != FR_OK) {
        overlay_status("Failed to mount USB drive\n");
        return -1;
    }

    int page = 0, pages, total;
    int active = 0;
    strncpy(pwd, dir, PWD_SIZE);
    while (1) {
        overlay_clear();
        int r = load_dir(pwd, page*PAGESIZE, PAGESIZE, &total, NULL);
        if (r == 0) {
            pages = (total+PAGESIZE-1) / PAGESIZE;
            overlay_status("Page ");
            overlay_printf("%d/%d", page+1, pages);
            if (active > file_len-1)
                active = file_len-1;
            for (int i = 0; i < PAGESIZE; i++) {
                int idx = page*PAGESIZE + i;
                overlay_cursor(2, i+TOPLINE);
                if (idx < total) {
                    overlay_printf(file_names[i]);
                    if (idx != 0 && file_dir[i])
                        overlay_printf("/");
                }
            }
            delay(300);
            while (1) {
                int r = joy_choice(TOPLINE, file_len, &active, OSD_KEY_CODE);
                if (r == 1 || r == 4) {
                    if (strcmp(pwd, dir) == 0 && page == 0 && active == 0) {
                        // return to main menu
                        return 1;
                    } else if (file_dir[active]) {
                        if (file_names[active][0] == '.' && file_names[active][1] == '.') {
                            // return to parent dir
                            // message(file_names[active], 1);
                            char *slash = strrchr(pwd, '/');
                            if (slash)
                                *slash = '\0';
                        } else {								// enter sub dir
                            strncat(pwd, "/", PWD_SIZE);
                            strncat(pwd, file_names[active], PWD_SIZE);
                        }
                        active = 0;
                        page = 0;
                        break;
                    } else {
                        // int res = 1;
                        strncpy(fname, pwd, 1024);
                        strncat(fname, "/", 1024);
                        strncat(fname, file_names[active], 1024);

                        joy1_state = 0; joy2_state = 0; // clear joypad states

                        // pwd determines the type of the ROM
                        if (prefix("usb:cores", pwd)) {
                            overlay_status("Core: %s", file_names[active]);
                            enable_jtag_pins();
                            load_core(fname);
                            _overlay_on = 1;                // turn on overlay after core is loaded
                            disable_jtag_pins();
                            return 0;       // return to main menu
                        } else {
                            bool success = false;
                            active_core = get_core_id();
                            // find core info entry
                            struct core_info *core = NULL;
                            for (int i = 0; core_info_list[i].id != 0; i++) {
                                if (prefix(core_info_list[i].rom_dir, pwd)) {
                                    core = &core_info_list[i];
                                    overlay_status("ROM for: %s", core->display_name);
                                }
                            }

                            // load core if needed
                            if (core != NULL) {
                                if (active_core != core->id) {
                                    char *fname_core = malloc(1024);
                                    if (!fname_core) {
                                        overlay_status("Failed to allocate memory");
                                        delay(1000);
                                        break;
                                    }
                                    if (find_core_for_board(fname_core, core->core_file)) {
                                        // load core
                                        enable_jtag_pins();
                                        load_core(fname_core);
                                        _overlay_on = 1;
                                        disable_jtag_pins();

                                        // allow 2 seconds for core to start
                                        uint64_t start = bflb_mtimer_get_time_ms();
                                        while (bflb_mtimer_get_time_ms() - start < 2000) {
                                            send_blank_packet();
                                            active_core = get_core_id();
                                            if (active_core == core->id)
                                                break;
                                        }
                                    } 
                                    free(fname_core);
                                }

                                if (active_core == core->id) {
                                    // Core is ready, load ROM
                                    overlay_status("Loading ROM: %s", file_names[active]);
                                    core->load_rom(fname);
                                    success = true;
                                    break;

                                    if (!success) {
                                        overlay_status("No core for: %s", pwd);
                                    } else {
                                        overlay_status("ROM loaded");
                                    }
                                    break;      // redraw file list
                                } else {
                                    overlay_status("Core failed to load");
                                    delay(1000);
                                    break;
                                }
                            }
                        }
                    }
                }
                if (r == 2 && page < pages-1) {
                    page++;
                    break;
                } else if (r == 3 && page > 0) {
                    page--;
                    break;
                }
                delay(10);
            }
        } else {
            overlay_status("Error opening director");
            overlay_printf(" %d", r);
            return -1;
        }
    }    

}

static void menu_options(void) {
    // to be implemented
}

// keep sending HID state to core until OSD is turned on
static void send_hid_to_core(void) {
    uint16_t hid1_old = 0, hid2_old = 0;
    bool first = true;
    overlay_status("Start sending HID to core...");
    while (1) {
        uint16_t joy1=0, joy2=0, hid1=0, hid2=0;    
        get_joypad_states(&joy1, &joy2, &hid1, &hid2);
        if (first || hid1 != hid1_old || hid2 != hid2_old) {    // send HID if changed
            bflb_uart_putchar(uart1_dev, 0x09);         
            bflb_uart_putchar(uart1_dev, hid1 & 0xff);
            bflb_uart_putchar(uart1_dev, hid1 >> 8);
            bflb_uart_putchar(uart1_dev, hid2 & 0xff);
            bflb_uart_putchar(uart1_dev, hid2 >> 8);
            hid1_old = hid1;
            hid2_old = hid2;
            first = false;
        }
        if (joy1 == OSD_KEY_CODE || joy2 == OSD_KEY_CODE || hid1 == OSD_KEY_CODE || hid2 == OSD_KEY_CODE) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    overlay_status("Stopped sending HID to core.");
}

// // (R L X A RT LT DN UP START SELECT Y B)
// Return 1 if a button was pressed, 0 otherwise
int joy_choice(int start_line, int len, int *active, int overlay_key_code) {
    if (*active < 0 || *active >= len)
        *active = 0;
    uint16_t joy1=0, joy2=0, hid1=0, hid2=0;    
    int last = *active;

    get_joypad_states(&joy1, &joy2, &hid1, &hid2);
    joy1 |= hid1;
    joy2 |= hid2;

    if ((joy1 == overlay_key_code) || (joy2 == overlay_key_code)) {
        overlay_status("OSD: %s", overlay_on() ? "ON" : "OFF");
        overlay(!overlay_on());    // toggle OSD
        delay(300);
    }

    if (!overlay_on()) {           // keep sending HID state to core when OSD is off
        send_hid_to_core();
        return 0;
    }

    if ((joy1 & 0x10) || (joy2 & 0x10)) {
        if (*active > 0) (*active)--;
    }
    if ((joy1 & 0x20) || (joy2 & 0x20)) {
        if (*active < len-1) (*active)++;
    }
    if ((joy1 & 0x40) || (joy2 & 0x40))
        return 3;      // previous page
    if ((joy1 & 0x80) || (joy2 & 0x80))
        return 2;      // next page
    if ((joy1 & 0x100) || (joy2 & 0x100))
        return 4;      // button A pressed
    if ((joy1 & 0x1) || (joy2 & 0x1))
        return 1;      // button B pressed

    overlay_cursor(0, start_line + (*active));
    overlay_printf(">");

    // overlay_cursor(0, 27);
    // overlay_printf(" j1=%04x j2=%04x h1=%04x h2=%04x", joy1, joy2, hid1, hid2);
    if (last != *active) {
        overlay_cursor(0, start_line + last);
        overlay_printf(" ");
        delay(100);     // button debounce
    }    
    return 0;
}

// null-terminated list of core info
struct core_info core_info_list[] = {
    {1, "NES", "usb:nes", "nestang.bin", loadnes},
    {2, "SNES", "usb:snes", "snestang.bin", loadsnes},
    {3, "Game Boy Advance", "usb:gba", "gbatang.bin", loadgba},
    {4, "MegaDrive / Genesis", "usb:genesis", "mdtang.bin", loadmd},
    {0, NULL, NULL, NULL, NULL}
};

// Main menu listing:
// >0: core id, -1: cores menu, -2: options menu, 0: end of list
int16_t main_menu_config[] = 
   {1,2,
#if defined(TANG_MEGA60K) || defined(TANG_MEGA138K) || defined(TANG_CONSOLE60K) || defined(TANG_CONSOLE138K)
    3,4,
#endif
    -1, -2, 0};

// Find a core file in the search order:
// usb:cores/${BOARD_NAME}/${core_name}
// usb:cores/${core_name}
bool find_core_for_board(char *fname, const char *core_name) {
    // check usb:cores/${BOARD_NAME}/${core_name}
    strncpy(fname, "usb:cores/", 1024);
    strncat(fname, BOARD_NAME, 1024);
    strncat(fname, "/", 1024);
    strncat(fname, core_name, 1024);
    FILINFO fno;
    if (f_stat(fname, &fno) == FR_OK && fno.fsize > 0) {
        return true;
    }

    // check usb:cores/${core_name}
    strncpy(fname, "usb:cores/", 1024);
    strncat(fname, core_name, 1024);
    if (f_stat(fname, &fno) == FR_OK && fno.fsize > 0) {
        return true;
    }

    return false;
}

#define MAIN_TASK_STACK_SIZE  2048
#define MAIN_TASK_PRIORITY    3
#define UART1_RX_TASK_STACK_SIZE  512
#define UART1_RX_TASK_PRIORITY    3

extern void fatfs_usbh_driver_register(void);

// Receive joypad updates and other UART responses from the FPGA
static void uart1_rx_task(void *pvParameters)
{
    uint8_t buffer[5];
    uint8_t pos = 0;
    uint8_t type = 0;
    
    while (1) {
        if (bflb_uart_rxavailable(uart1_dev)) {
            uint8_t ch = bflb_uart_getchar(uart1_dev);
            
            if ((ch == 0x01 || ch == 0x11) && pos == 0) {        // Start of new packet
                pos = 1;
                type = ch;
            } else if (type == 0x1 && pos > 0 && pos < 5) {      // periodic joypad state
                buffer[pos-1] = ch;
                pos++;
                
                // Complete packet received
                if (pos == 5) {
                    // Combine bytes into 16-bit values
                    uint16_t joy1 = (buffer[1] << 8) | buffer[0];
                    uint16_t joy2 = (buffer[3] << 8) | buffer[2];
                    
                    // Update global state with mutex protection
                    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
                        joy1_state = joy1;
                        joy2_state = joy2;
                        xSemaphoreGive(state_mutex);
                    }
                    
                    pos = 0; // Reset for next packet
                }
            } else if (type == 0x11 && pos == 1) {           // response to command 1 (get core ID)
                if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
                    core_id = ch;
                    xSemaphoreGive(state_mutex);
                }
                pos = 0;
            } else {
                pos = 0; // Reset if we get out of sync
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Display main menu and call other menu functions
static void main_task(void *pvParameters)
{
    uint32_t last_redraw_time = 0;
    // volatile uint32_t *reg_gpio0 = (volatile uint32_t *)0x200008c4;     // bl616 reference 4.8.5
    // volatile uint32_t *reg_gpio1 = (volatile uint32_t *)0x200008c8;
    // volatile uint32_t *reg_gpio2 = (volatile uint32_t *)0x200008cc;
    // volatile uint32_t *reg_gpio3 = (volatile uint32_t *)0x200008d0;

    // wait for USB drive to be ready
    overlay_status("Waiting for USB drive...");
    uint64_t start = bflb_mtimer_get_time_ms();
    while (f_mount(&fs, "usb:", 1) != FR_OK) {
        delay(100);
    }
    overlay_status("USB drive mounted in %d ms", bflb_mtimer_get_time_ms() - start);
    
    // load monitor core at startup
    enable_jtag_pins();
    if (find_core_for_board(fname, "monitor.bin")) {
        load_core(fname);
    } else {
        overlay_status("No monitor.bin found for board.");
    }
    disable_jtag_pins();

    int line_start;
    int menu_cnt = 0;
    for (int i = 0; main_menu_config[i] != 0; i++) {
        menu_cnt++;
    }
    line_start = 13 - (menu_cnt+2+2) / 2;       // 2 lines for version, 2 lines for "TangCore"

    while (1) {
        bool redraw = true;
        int choice = 0;
        for (;;) {
            uint32_t now = bflb_mtimer_get_time_ms();
            if (active_core == -1) {
                send_blank_packet();
                active_core = get_core_id();            // 200ms timeout
                if (active_core >= 0) redraw = true;    // redraw immediately if core is detected
            }
            // if (core < 0) continue;         // do not draw or process input if core is not ready
            if (now - last_redraw_time > 5000) 
                redraw = true;
            if (redraw) {
                active_core = get_core_id();            // allow jtag to change core underneath us
                overlay(overlay_on());                  // set correct overlay state
                overlay_clear();

                int line = line_start;
                overlay_cursor(0, line++);
                //              01234567890123456789012345678901
                overlay_printf("       -== TangCore ==-");
                line++;

                // display all menu items
                for (int i = 0; i < menu_cnt; i++) {
                    overlay_cursor(2, line++);
                    if (main_menu_config[i] > 0) {
                        for (int j = 0; core_info_list[j].id != 0; j++) {
                            if (core_info_list[j].id == main_menu_config[i]) {
                                overlay_printf("%s", core_info_list[j].display_name);
                                break;
                            }
                        }
                    } else if (main_menu_config[i] == -1) {
                        overlay_printf("Cores");
                    } else if (main_menu_config[i] == -2) {
                        overlay_printf("Options");
                    }
                }

                line++;
                overlay_cursor(2, line++);
                overlay_printf("Version: ");
                overlay_printf(__DATE__);
                last_redraw_time = now;
                redraw = false;

                // print some debug stats to UART
                // uint16_t joy1=0, joy2=0;
                // get_joypad_states(&joy1, &joy2);
                // overlay_status("core=%d, j1=%04x, j2=%04x", active_core, joy1, joy2);
                // overlay_status("Mtimer frequency: %d MHz", bflb_mtimer_get_freq() / 1000000);
                // overlay_status("CPU frequency: %d MHz", bflb_clk_get_system_clock(BL_SYSTEM_CLOCK_MCU_CLK) / 1000000);
                // overlay_status("GPIO0-3 status: %08x %08x %08x %08x", *reg_gpio0, *reg_gpio1, *reg_gpio2, *reg_gpio3);
            }

            int r = joy_choice(line_start+2, menu_cnt, &choice, OSD_KEY_CODE);
            if (r == 1) break;

            delay(20);
        }

        if (main_menu_config[choice] > 0) {
            // Load rom or core from USB drive
            struct core_info *core = NULL;
            for (int i = 0; core_info_list[i].id != 0; i++) {
                if (core_info_list[i].id == main_menu_config[choice]) {
                    core = &core_info_list[i];
                    break;
                }
            }
            if (core) 
                menu_loadrom(core->rom_dir);
        } else if (main_menu_config[choice] == -1) {
            // load cores manually
            menu_loadrom("usb:cores");
        } else if (main_menu_config[choice] == -2) {
            // Options
            menu_options();
        } 

        delay(300);
    }
}

static void print_system_info(void) {
    // this is viewable with scripts/liveuart.py
    overlay_status("TangCore %s", __DATE__);
    overlay_status("TangBoard: %s", BOARD_NAME);
    overlay_status("System clock: %u MHz", bflb_clk_get_system_clock(BL_SYSTEM_CLOCK_MCU_CLK) / 1000000);
    // UART registers
    // Clock comes from XCLK/160M/BCLK and goes through a divider and becomes UART_CLK
    overlay_status("GLB_UART_CFG0: %08x", BL_RD_WORD(0x20000150));
    overlay_status("GLB_UART_CFG1: %08x", BL_RD_WORD(0x20000154));
    overlay_status("GLB_UART_CFG2: %08x", BL_RD_WORD(0x20000158));
    overlay_status("UART0 clock: %u", Clock_Peripheral_Clock_Get(BL_PERIPHERAL_CLOCK_UART0));
    overlay_status("UART1 clock: %u", Clock_Peripheral_Clock_Get(BL_PERIPHERAL_CLOCK_UART1));

    // 10.3.5: baudrate = UART_clk / (uart_prd + 1)
    // This causes memory exception.
    // overlay_status("UART_BIT_PRD: %08x", BL_RD_WORD(0x40010008));
    //              01234567890123456789012345678901
    // overlay_status("                                ");
}

// Initialize things, then start main_task and uart1_rx_task to do actual work
int main(void)
{
    /* Board init */
    board_init();
    
    // Initialize GPIO and UART
    init_gpio_and_uart();

    print_system_info();

    // Create mutex for joypad states
    state_mutex = xSemaphoreCreateMutex();

    overlay_status("Initializing USB host...");

    // Initializing USB host...
    usbh_initialize();
    fatfs_usbh_driver_register();
    usb_gamepad_init();

    overlay_status("Creating tasks...");
    // Create the tasks
    xTaskCreate(main_task, "main_task", MAIN_TASK_STACK_SIZE, NULL, MAIN_TASK_PRIORITY, &main_task_handle);
    xTaskCreate(uart1_rx_task, "uart1_rx_task", UART1_RX_TASK_STACK_SIZE, NULL, UART1_RX_TASK_PRIORITY, &uart1_rx_task_handle);
    
    vTaskStartScheduler();

    while (1) {
    }
}
