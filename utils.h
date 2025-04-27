#pragma once

#include <strings.h>
#include "bflb_gpio.h"
#include "bflb_uart.h"

#include <FreeRTOS.h>
#include "task.h"
#include "semphr.h"

// #include "usbh_core.h"
#include "ff.h"

#define DEBUG(...) overlay_printf(__VA_ARGS__)
// #define DEBUG(...) do {} while(0)

extern struct bflb_device_s *gpio_dev;

#if defined(TANG_NANO20K)
#define GPIO_PIN_JTAG_TMS GPIO_PIN_16
#define GPIO_PIN_JTAG_TCK GPIO_PIN_10
#define GPIO_PIN_JTAG_TDI GPIO_PIN_14
#define GPIO_PIN_JTAG_TDO GPIO_PIN_12
#else
#define GPIO_PIN_JTAG_TMS GPIO_PIN_0
#define GPIO_PIN_JTAG_TCK GPIO_PIN_1
#define GPIO_PIN_JTAG_TDI GPIO_PIN_3
#define GPIO_PIN_JTAG_TDO GPIO_PIN_2
#endif

extern volatile uint32_t *reg_gpio_tms;
extern volatile uint32_t *reg_gpio_tck;
extern volatile uint32_t *reg_gpio_tdo;
extern volatile uint32_t *reg_gpio_tdi;

#define DERIVE_GPIO_OPS_OUT(GPIO_PIN_XXX)        \
    static inline void GPIO_PIN_XXX##_H(void)    \
    {                                            \
        bflb_gpio_set(gpio_dev, GPIO_PIN_XXX);   \
    }                                            \
                                                 \
    static inline void GPIO_PIN_XXX##_L(void)    \
    {                                            \
        bflb_gpio_reset(gpio_dev, GPIO_PIN_XXX); \
    }                                            \
    static inline void GPIO_PIN_XXX##_W(bool s)  \
    {                                            \
        if (s)                                   \
            GPIO_PIN_XXX##_H();                  \
        else                                     \
            GPIO_PIN_XXX##_L();                  \
    }

#define DERIVE_GPIO_OPS_IN(GPIO_PIN_XXX)               \
    static inline bool GPIO_PIN_XXX##_V(void)          \
    {                                                  \
        return bflb_gpio_read(gpio_dev, GPIO_PIN_XXX); \
    }

DERIVE_GPIO_OPS_OUT(GPIO_PIN_JTAG_TMS);
DERIVE_GPIO_OPS_OUT(GPIO_PIN_JTAG_TCK);
DERIVE_GPIO_OPS_OUT(GPIO_PIN_JTAG_TDI);
DERIVE_GPIO_OPS_IN(GPIO_PIN_JTAG_TDO);

#include <string.h>

#ifndef max
#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })
#endif

#ifndef min 
#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })
#endif

static inline bool prefix(const char *pre, const char *str)
{
    return strncasecmp(pre, str, strlen(pre)) == 0;
}

void overlay_status(const char *fmt, ...);
void overlay_printf(const char *fmt, ...);
void overlay_clear(void);
void overlay_cursor(int x, int y);
int overlay_on(void);
void overlay(int on);
void dprint(const char *fmt, ...);

// return true if core is ready. then core_id is set.
// return false if timeout after 100ms
bool get_core_status(void);
// read joypad states, joy1/2 comes from FPGA, hid1/2 comes from USB
void get_joypad_states(uint16_t *joy1, uint16_t *joy2, uint16_t *hid1, uint16_t *hid2);
extern int joy_choice(int start_line, int len, int *active, int overlay_key_code);
extern void send_blank_packet(void);

extern void bflb_uart_set_console(struct bflb_device_s *dev);
extern char *strcasestr(const char *haystack, const char *needle);

static inline void delay(uint32_t ms)
{
#if defined(TANG_CONSOLE60K) || defined(TANG_CONSOLE138K)
    vTaskDelay(pdMS_TO_TICKS(ms));
#else
    // compensate for 26MHz clock instead of 40MHz
    vTaskDelay(pdMS_TO_TICKS(ms*26/40));
#endif
}

struct core_info {
    uint16_t id;                    // 1: NES, 2: SNES, 3: GB, 4: GENESIS, 0: end
    const char *display_name;
    const char *rom_dir;            // usb:nes, usb:snes, etc.
    const char *core_file;          // core file in cores/
    int (*load_rom)(const char *fname);
};

#define OPTION_OSD_KEY_SELECT_START 1
#define OPTION_OSD_KEY_SELECT_RIGHT 2

extern int option_osd_key;
#define OSD_KEY_CODE (option_osd_key == OPTION_OSD_KEY_SELECT_START ? 0xC : 0x84)

extern int loadpc(const char *fname);
extern void set_loading_state(int state);
extern void send_fbuf_data(uint16_t len);
extern void overlay_message(const char *msg, int center);

#ifndef USB_NOCACHE_RAM_SECTION
#define USB_NOCACHE_RAM_SECTION __attribute__((section(".noncacheable")))
#endif

extern bool core_running;
extern USB_NOCACHE_RAM_SECTION FIL fcore;
extern USB_NOCACHE_RAM_SECTION FIL ffloppy;
#define BLOCK_SIZE (8*1024)
extern USB_NOCACHE_RAM_SECTION BYTE __attribute__((aligned(64))) fbuf[BLOCK_SIZE];
extern bool floppy_mounted;

extern struct bflb_device_s *uart1_dev;

// len: length of payload including the command (>=1)
extern void fpga_tx_header(int cmd, int len);
extern void fpga_tx_byte(uint8_t b);

// inline void fpga_tx(int cmd, uint16_t len, int *data) {
//     bflb_uart_putchar(uart1_dev, 0xAA);
//     bflb_uart_putchar(uart1_dev, len >> 8);
//     bflb_uart_putchar(uart1_dev, len & 0xFF);
//     for (int i = 0; i < len; i++) {
//         bflb_uart_putchar(uart1_dev, data[i]);
//     }
// }

