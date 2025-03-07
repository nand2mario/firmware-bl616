#pragma once

#include "bflb_gpio.h"

#include <FreeRTOS.h>
#include "task.h"
#include "semphr.h"

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

// return true if core is ready. then core_id is set.
// return false if timeout after 100ms
bool get_core_status(void);
void get_joypad_states(uint16_t *joy1, uint16_t *joy2);
extern int joy_choice(int start_line, int len, int *active, int overlay_key_code);
extern void send_blank_packet(void);
bool find_core_for_board(char *fname, const char *core_name);

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
extern struct core_info core_info_list[];
extern int16_t main_menu_config[];

#define OPTION_OSD_KEY_SELECT_START 1
#define OPTION_OSD_KEY_SELECT_RIGHT 2

#define OSD_KEY_CODE (option_osd_key == OPTION_OSD_KEY_SELECT_START ? 0xC : 0x84)
