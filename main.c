#include <FreeRTOS.h>
#include <stdarg.h>
#include "semphr.h"
#include "usbh_core.h"
#include "bflb_gpio.h"
#include "bflb_uart.h"
#include "board.h"
#include "bl616_glb.h"
#include "task.h"
#include "ff.h"

#include "programmer.h"
#include "utils.h"

// UART
struct bflb_device_s *gpio_dev;
struct bflb_device_s *uart0_dev;
struct bflb_device_s *uart1_dev;

extern void bflb_uart_set_console(struct bflb_device_s *dev);
int joy_choice(int start_line, int len, int *active);

// USB and fatfs
struct usbh_msc *msc;
char load_fname[1024];

// Add function prototype at the top
bool uart0_active = false;

// Add these global variables at the top with other globals
volatile uint16_t joy1_state = 0;
volatile uint16_t joy2_state = 0;
SemaphoreHandle_t joy_mutex;

void get_joypad_states(uint16_t *joy1, uint16_t *joy2);

static void init_gpio_and_uart(void)
{
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

    // JTAG pins
    bflb_gpio_init(gpio_dev, GPIO_PIN_JTAG_TMS, GPIO_OUTPUT | GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_3);
    bflb_gpio_init(gpio_dev, GPIO_PIN_JTAG_TCK, GPIO_OUTPUT | GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_3);
    bflb_gpio_init(gpio_dev, GPIO_PIN_JTAG_TDI, GPIO_OUTPUT | GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_3);
    bflb_gpio_init(gpio_dev, GPIO_PIN_JTAG_TDO, GPIO_INPUT  | GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_3);

    /* Core control UART 1 */
    bflb_gpio_uart_init(gpio_dev, GPIO_PIN_28, GPIO_UART_FUNC_UART1_TX);    // JTAG connector pin 6
    bflb_gpio_uart_init(gpio_dev, GPIO_PIN_27, GPIO_UART_FUNC_UART1_RX);    // JTAG connector pin 7 (pin8 is GND, pin1 is VCC)

    /* Set up Core control UART parameters */
    struct bflb_uart_config_s uart1_cfg = {
        .baudrate = 1000000,
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

    /* Set up Debug UART parameters */
    /* Debug UART 0 */
    // bflb_gpio_uart_init(gpio_dev, GPIO_PIN_2, GPIO_UART_FUNC_UART0_RX);     // JTAG TDO
    // if (uart0_active) {
    //     bflb_gpio_uart_init(gpio_dev, GPIO_PIN_3, GPIO_UART_FUNC_UART0_TX);     // JTAG TDI
    // }
    // struct bflb_uart_config_s uart0_cfg = {
    //     .baudrate = 115200,    // 115200 baud
    //     .data_bits = UART_DATA_BITS_8,
    //     .stop_bits = UART_STOP_BITS_1,
    //     .parity    = UART_PARITY_NONE,
    //     .tx_fifo_threshold = 7,
    //     .rx_fifo_threshold = 7,
    //     .flow_ctrl = 0,  /* No CTS/RTS flow control */
    // };
    /* Get handle to UART0 */
    // uart0_dev = bflb_device_get_by_name("uart0");
    /* Initialize UART1 with the config */
    // bflb_uart_init(uart0_dev, &uart0_cfg);
    /* Redirect standard I/O to UART1 */
    // bflb_uart_set_console(uart0_dev);
}

static uint32_t uart0_last_time;
// If uart0 is not used for 0.5 second, turn it off to allow JTAG operations
static void uart0_timeout(void) {
    uint32_t now = bflb_mtimer_get_time_ms();
    if (now - uart0_last_time > 500) {
        // bflb_gpio_deinit(gpio_dev, GPIO_PIN_3);
        // bflb_gpio_deinit(gpio_dev, GPIO_PIN_2);
        uart0_active = false;
    }
}

static void uart0_on(void) {
    if (!uart0_active) {
        bflb_gpio_uart_init(gpio_dev, GPIO_PIN_3, GPIO_UART_FUNC_UART0_TX);     // JTAG TDI
        // bflb_gpio_uart_init(gpio_dev, GPIO_PIN_2, GPIO_UART_FUNC_UART0_RX);     // JTAG TDO
        uart0_active = true;
    }
    uart0_last_time = bflb_mtimer_get_time_ms();
}

void debug_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    char buf[256];
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    // uart0_on();
    for (int i = 0; buf[i] != '\0' && i < sizeof(buf); i++) 
        bflb_uart_putchar(uart0_dev, buf[i]);
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

    overlay_cursor(0, 27);
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
    vTaskDelay(pdMS_TO_TICKS(300));
    for (;;) {
        uint16_t joy1, joy2;
        get_joypad_states(&joy1, &joy2);
           if ((joy1 & 0x1) || (joy1 & 0x100) || (joy2 & 0x1) || (joy2 & 0x100))
               break;
    }
    vTaskDelay(pdMS_TO_TICKS(300));
}

const char *ROM_DIR[] = {
    "usb:nes",
    "usb:snes",
    "usb:gba",
    "usb:genesis",
    "usb:cores"
};

#include "ff.h"

// nand2mario: these USB data structures cannot be CACHED as they are written to by hardware
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
int file_dir[PAGESIZE];
int file_sizes[PAGESIZE];
int file_len;		// number of files on this page

bool load_core_crc(char *fname) {
    uint16_t crc = 0;
    UINT bytes_read;
    FRESULT res_sd = f_mount(&fs, "usb:", 1);
    if (res_sd != FR_OK) {
        overlay_printf("mount fail, res:%d\r\n", res_sd);
        return false;
    }
    res_sd = f_open(&fcore, fname, FA_READ);
    if (res_sd != FR_OK) {
        overlay_printf("open fail, res:%d\r\n", res_sd);
        return false;
    }
    bool res = false;
    for(;;) {
        f_read(&fcore, fbuf, BLOCK_SIZE, &bytes_read);
        overlay_status("f_read: offset=%u, bytes=%d, 4 bytes=%02x %02x %02x %02x", 
            (uint32_t)f_tell(&fcore), bytes_read, fbuf[0], fbuf[1], fbuf[2], fbuf[3]);
        if (bytes_read == 0) break;
        for (int i = 0; i < bytes_read; i++)
            crc = update_crc_16(crc, fbuf[i]);
        if (bytes_read < BLOCK_SIZE) break;
    }
    overlay_status("CRC16: %04x", crc);
    vTaskDelay(pdMS_TO_TICKS(2000));

load_core_close:
    f_close(&fcore);
    return true;
}

bool load_core(char *fname) {
    FRESULT res_sd = f_mount(&fs, "usb:", 1);
    if (res_sd != FR_OK) {
        overlay_printf("mount fail, res:%d\r\n", res_sd);
        return false;
    }
    FILINFO fno;
    f_stat(fname, &fno);
    int len = fno.fsize;
    overlay_status("Writing %u bytes...", len);

    res_sd = f_open(&fcore, fname, FA_READ);
    if (res_sd != FR_OK) {
        overlay_printf("open fail, res:%d\r\n", res_sd);
        return false;
    }
    bool res = false;

    chain_len = detectChain(JTAG_MAX_CHAIN);
    if (chain_len == 0 || (idcodes[0] != IDCODE_GW5AT_60 && idcodes[0] != IDCODE_GWAST_138)) {
        overlay_printf("No GW5AT-60 or GW5AST-138 detected\n");
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

    UINT bytes = 0, total = 0;
    taskENTER_CRITICAL();
    uint64_t time_total = bflb_mtimer_get_time_us();
    uint64_t time_jtag = 0, time_flash = 0;
    extern uint64_t jtag_writetdi_time;
    uint64_t writetdi_time_start = jtag_writetdi_time;
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

    time_total = bflb_mtimer_get_time_us() - time_total;
    overlay_status("Time: total=%lld us, jtag=%lld us, flash=%lld us, writetdi=%lld us", time_total, time_jtag, 
        time_flash, jtag_writetdi_time - writetdi_time_start);

    // printf("Status after program sram: %x\n", readStatusReg());
    res = true;

free_fbuf_cached:
    free(fbuf_cached);

load_core_close:
    f_close(&fcore);
    return res;
}


// load core from embedded flash
int load_core_flash() {
    uint32_t addr = 0x100000;
    uint32_t len = 2410656;
    uint8_t *buf = NULL;
    bool res = false;
    buf = malloc(8*1024);   // 8KB buffer
    if (!buf) {
        overlay_status("Cannot malloc buffer\r\n");
        return 0;
    }

    if (chain_len == 0)
        chain_len = detectChain(JTAG_MAX_CHAIN);
    if (chain_len == 0 || idcodes[0] != IDCODE_GW5AT_60 && idcodes[0] != IDCODE_GWAST_138) {
        overlay_status("No GW5AT-60 or GW5AST-138 detected\n");
        goto program_free;
    }

    if (!eraseSRAM()) {
        overlay_status("Failed to erase SRAM\n");
        goto program_free;
    }

    if (!eraseSRAM()) {
        overlay_status("Failed to erase SRAM 2nd time\n");
        goto program_free;
    }    

    if (!writeSRAM_start()) {
        overlay_status("Failed to start write SRAM\n");
        goto program_free;
    }

    taskENTER_CRITICAL();
    uint64_t time_total = bflb_mtimer_get_time_us();
    uint64_t time_jtag = 0, time_flash = 0;
    for (int i = 0; i < len; i += 8*1024) {
        uint64_t time_flash_start = bflb_mtimer_get_time_us();
        bflb_flash_read(addr + i, buf, 8*1024);
        time_flash += bflb_mtimer_get_time_us() - time_flash_start;
        int bytes = len - i < 8*1024 ? len - i : 8*1024;

        // reverse msb/lsb as Gowin bitstream needs to MSB first
        static unsigned char lookup[16] = {
            0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
            0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf, };
        for (int j = 0; j < bytes; j++)
            buf[j] = lookup[buf[j] & 0xf] << 4 | lookup[buf[j] >> 4];

        bool last = i + bytes >= len;
        uint64_t time_jtag_start = bflb_mtimer_get_time_us();
        if (!writeSRAM_send(buf, bytes*8, last)) {
            overlay_status("Failed to send data to SRAM\n");
            goto program_free;
        }
        time_jtag += bflb_mtimer_get_time_us() - time_jtag_start;
    }
    if (!writeSRAM_end()) {
        overlay_status("Failed to program SRAM\n");
        goto program_free;
    }
    taskEXIT_CRITICAL();

    time_total = bflb_mtimer_get_time_us() - time_total;
    overlay_status("Time: total=%lld us, jtag=%lld us, flash=%lld us", time_total, time_jtag, time_flash);

    // printf("Status after program sram: %x\n", readStatusReg());
    res = true;

program_free:
    free(buf);    
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

// dir: initial dir
// return 0: user chose a ROM (*choice), 1: no choice made, -1: error
// file chosen: pwd / file_name[*choice]
static int menu_loadrom(char *dir, int *choice) {
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
            vTaskDelay(pdMS_TO_TICKS(300));
            while (1) {
                int r = joy_choice(TOPLINE, file_len, &active /*, OSD_KEY_CODE */);
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
                        // actually load a ROM
                        *choice = active;
                        // int res = 1;
                        
                        // todo: actually load core and rom
                        // pwd determines the type of the ROM
                        if (prefix("usb:cores", pwd)) {
                            strncpy(load_fname, pwd, 1024);
                            strncat(load_fname, "/", 1024);
                            strncat(load_fname, file_names[active], 1024);
                            overlay_status("Core: %s", file_names[active]);
                            if (r == 1)
                                load_core(load_fname);
                            else
                                load_core_crc(load_fname);
                            return 0;       // return to main menu
                        } else {
                            overlay_message("ROM loading not implemented",1);
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

// // (R L X A RT LT DN UP START SELECT Y B)
// Return 1 if a button was pressed, 0 otherwise
int joy_choice(int start_line, int len, int *active) {
    if (*active < 0 || *active >= len)
        *active = 0;
    uint16_t joy1, joy2;    
    int last = *active;

    get_joypad_states(&joy1, &joy2);

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

    overlay_cursor(0, 27);
    // overlay_printf(" joy1=%04x, joy2=%04x", joy1, joy2);
    if (last != *active) {
        overlay_cursor(0, start_line + last);
        overlay_printf(" ");
        vTaskDelay(pdMS_TO_TICKS(100));     // button debounce
    }    
    return 0;
}

static void main_task(void *pvParameters)
{
    uint32_t last_redraw_time = 0;
    volatile uint32_t *reg_gpio0 = (volatile uint32_t *)0x200008c4;     // bl616 reference 4.8.5
    volatile uint32_t *reg_gpio1 = (volatile uint32_t *)0x200008c8;
    volatile uint32_t *reg_gpio2 = (volatile uint32_t *)0x200008cc;
    volatile uint32_t *reg_gpio3 = (volatile uint32_t *)0x200008d0;

    while (1) {
        bool redraw = true;
        int choice = 0;
        for (;;) {
            uint32_t now = bflb_mtimer_get_time_ms();
            if (now - last_redraw_time > 5000) 
                redraw = true;
            if (redraw) {
                overlay_clear();
                overlay_cursor(2, 9);
                //              01234567890123456789012345678901
                overlay_printf("=== Welcome to TangCore ===\r\n");

                overlay_cursor(2, 11);
                overlay_printf("NES\n");
                overlay_cursor(2, 12);
                overlay_printf("SNES");
                overlay_cursor(2, 13);
                overlay_printf("GBA");
                overlay_cursor(2, 14);
                overlay_printf("MegaDrive / Genesis");
                overlay_cursor(2, 15);
                overlay_printf("Cores");
                overlay_cursor(2, 16);
                overlay_printf("Load flash core");
                
                overlay_cursor(2, 18);
                overlay_printf("Version: ");
                overlay_printf(__DATE__);
                last_redraw_time = now;
                redraw = false;

                // print some debug stats to UART
                // overlay_status("Mtimer frequency: %d MHz", bflb_mtimer_get_freq() / 1000000);
                // overlay_status("CPU frequency: %d MHz", bflb_clk_get_system_clock(BL_SYSTEM_CLOCK_MCU_CLK) / 1000000);
                // overlay_status("GPIO0-3 status: %08x %08x %08x %08x", *reg_gpio0, *reg_gpio1, *reg_gpio2, *reg_gpio3);
            }

            // uint16_t joy1, joy2;
            // get_joypad_states(&joy1, &joy2);
            // overlay_cursor(0, 27);
            // overlay_printf("joy1=%04x, joy2=%04x", joy1, joy2);

            // uart0_timeout();        // necessary to keep JTAG alive

            int r = joy_choice(11, 6, &choice);
            if (r == 1) break;

            // vTaskDelay(pdMS_TO_TICKS(200));
        }

        if (choice <= 4) {  // 0: NES, 1: SNES, 2: GBA, 3: MegaDrive / Genesis, 4: Cores
            // Load rom or core from USB drive
            menu_loadrom(ROM_DIR[choice], &choice);
        } else {
            // Options
            load_core_flash();
        } 

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

#define MAIN_TASK_STACK_SIZE  2048
#define MAIN_TASK_PRIORITY    3
#define UART1_RX_TASK_STACK_SIZE  512
#define UART1_RX_TASK_PRIORITY    3
static TaskHandle_t main_task_handle;
static TaskHandle_t uart1_rx_task_handle;

extern void fatfs_usbh_driver_register(void);

// Receive joypad states from the FPGA
static void uart1_rx_task(void *pvParameters)
{
    uint8_t buffer[5];
    uint8_t pos = 0;
    
    while (1) {
        if (bflb_uart_rxavailable(uart1_dev)) {
            uint8_t ch = bflb_uart_getchar(uart1_dev);
            
            // Start of new packet
            if (ch == 0x01 && pos == 0) {
                pos = 1;
            }
            // Collect packet data
            else if (pos > 0 && pos < 5) {
                buffer[pos-1] = ch;
                pos++;
                
                // Complete packet received
                if (pos == 5) {
                    // Combine bytes into 16-bit values
                    uint16_t joy1 = (buffer[1] << 8) | buffer[0];
                    uint16_t joy2 = (buffer[3] << 8) | buffer[2];
                    
                    // Update global state with mutex protection
                    if (xSemaphoreTake(joy_mutex, portMAX_DELAY) == pdTRUE) {
                        joy1_state = joy1;
                        joy2_state = joy2;
                        xSemaphoreGive(joy_mutex);
                    }
                    
                    pos = 0; // Reset for next packet
                }
            }
            else {
                pos = 0; // Reset if we get out of sync
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

int main(void)
{
    /* Board init */
    board_init();
    
    // Initialize GPIO and UART
    init_gpio_and_uart();

    // Create mutex for joypad states
    joy_mutex = xSemaphoreCreateMutex();

    // Initializing USB host...
    usbh_initialize(0, USB_BASE);
    fatfs_usbh_driver_register();

    // Create the tasks
    xTaskCreate(main_task, "main_task", MAIN_TASK_STACK_SIZE, NULL, MAIN_TASK_PRIORITY, &main_task_handle);
    xTaskCreate(uart1_rx_task, "uart1_rx_task", UART1_RX_TASK_STACK_SIZE, NULL, UART1_RX_TASK_PRIORITY, &uart1_rx_task_handle);
    
    vTaskStartScheduler();

    while (1) {
    }
}

// Add this function to safely read the joypad states
void get_joypad_states(uint16_t *joy1, uint16_t *joy2)
{
    if (xSemaphoreTake(joy_mutex, portMAX_DELAY) == pdTRUE) {
        *joy1 = joy1_state;
        *joy2 = joy2_state;
        xSemaphoreGive(joy_mutex);
    }
}
