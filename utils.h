#pragma once

#include "bflb_gpio.h"

extern struct bflb_device_s *gpio_dev;

#define GPIO_PIN_JTAG_TMS GPIO_PIN_0
#define GPIO_PIN_JTAG_TCK GPIO_PIN_1
#define GPIO_PIN_JTAG_TDI GPIO_PIN_3
#define GPIO_PIN_JTAG_TDO GPIO_PIN_2

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
