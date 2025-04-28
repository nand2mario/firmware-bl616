#pragma once

#include <stdint.h>

typedef struct {
    int len;
    uint8_t code[8];
} ps2_scancode_t;

// convert USB scancode to PS/2 scancode (set 2)
// is_break: true if the code is a break code, false if it is a make code
extern ps2_scancode_t usb_to_ps2(uint8_t usb_code, bool is_break);

