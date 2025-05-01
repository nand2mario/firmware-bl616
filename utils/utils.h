#pragma once

#include <strings.h>
#include <string.h>

extern "C" {
#include "bflb_gpio.h"
#include "bflb_uart.h"

#include <FreeRTOS.h>
#include "task.h"
#include "semphr.h"

// #include "usbh_core.h"
#include "ff.h"
}

#define DEBUG(...) dprint(__VA_ARGS__)
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

// "sd:" or "usb:"
extern const char *drv;

static inline bool prefix(const char *pre, const char *str)
{
    return strncasecmp(pre, str, strlen(pre)) == 0;
}
extern "C" char *strcasestr(const char *haystack, const char *needle);

// return true if core is ready. then core_id is set.
// return false if timeout after 100ms
bool get_core_status(void);
// read joypad states, joy1/2 comes from FPGA, hid1/2 comes from USB
void get_joypad_states(uint16_t *joy1, uint16_t *joy2, uint16_t *hid1, uint16_t *hid2);
extern int joy_choice(int start_line, int len, int *active, int overlay_key_code);
extern void send_blank_packet(void);

static inline void delay(uint32_t ms)
{
#if defined(TANG_CONSOLE60K) || defined(TANG_CONSOLE138K)
    vTaskDelay(pdMS_TO_TICKS(ms));
#else
    // compensate for 26MHz clock instead of 40MHz
    vTaskDelay(pdMS_TO_TICKS(ms*26/40));
#endif
}

#define OPTION_OSD_KEY_SELECT_START 1
#define OPTION_OSD_KEY_SELECT_RIGHT 2

extern int option_osd_key;
#define OSD_KEY_CODE (option_osd_key == OPTION_OSD_KEY_SELECT_START ? 0xC : 0x84)

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
extern bool mounted_a;

extern struct bflb_device_s *uart1_dev;

// len: length of payload including the command (>=1)
extern void fpga_tx_header(int cmd, int len);
extern void fpga_tx_byte(uint8_t b);

extern uint32_t get_file_size(const char *fname);

// Send a romdata packet to core of len bytes in `fbuf`
extern void send_fbuf_data(uint16_t len);
extern void send_blank_packet(void);

extern SemaphoreHandle_t state_mutex;              
extern volatile uint16_t joy1_state;
extern volatile uint16_t joy2_state;
extern volatile uint16_t hid1_state;
extern volatile uint16_t hid2_state;
extern volatile int16_t core_id;

extern void get_joypad_states(uint16_t *joy1, uint16_t *joy2, uint16_t *hid1, uint16_t *hid2);
extern int16_t get_core_id(void);

extern const char *cstr_find_ignore_case(const char *str, const char *substr);
