#include <map>
#include <vector>

#include "usb2ps2.h"

using namespace std;

static map<uint8_t, vector<uint8_t>> usbToPs2MakeMap;

void init_usb_to_ps2_map() {
    usbToPs2MakeMap = {
        // Alphanumeric keys
        {0x04, {0x1C}}, // a
        {0x05, {0x32}}, // b
        {0x06, {0x21}}, // c
        {0x07, {0x23}}, // d
        {0x08, {0x24}}, // e
        {0x09, {0x2B}}, // f
        {0x0A, {0x34}}, // g
        {0x0B, {0x33}}, // h
        {0x0C, {0x43}}, // i
        {0x0D, {0x3B}}, // j
        {0x0E, {0x42}}, // k
        {0x0F, {0x4B}}, // l
        {0x10, {0x3A}}, // m
        {0x11, {0x31}}, // n
        {0x12, {0x44}}, // o
        {0x13, {0x4D}}, // p
        {0x14, {0x15}}, // q
        {0x15, {0x2D}}, // r
        {0x16, {0x1B}}, // s
        {0x17, {0x2C}}, // t
        {0x18, {0x3C}}, // u
        {0x19, {0x2A}}, // v
        {0x1A, {0x1D}}, // w
        {0x1B, {0x22}}, // x
        {0x1C, {0x35}}, // y
        {0x1D, {0x1A}}, // z

        {0x1E, {0x16}}, // 1
        {0x1F, {0x1E}}, // 2
        {0x20, {0x26}}, // 3
        {0x21, {0x25}}, // 4
        {0x22, {0x2E}}, // 5
        {0x23, {0x36}}, // 6
        {0x24, {0x3D}}, // 7
        {0x25, {0x3E}}, // 8
        {0x26, {0x46}}, // 9
        {0x27, {0x45}}, // 0
        
        // Function and modifier keys
        {0x28, {0x5A}}, // Enter (Main)
        {0x29, {0x76}}, // Escape
        {0x2A, {0x66}}, // Backspace
        {0x2B, {0x0D}}, // Tab
        {0x2C, {0x29}}, // Space
        {0x2D, {0x4E}}, // - (Minus)
        {0x2E, {0x55}}, // = (Equals)
        {0x2F, {0x54}}, // [ (Left Bracket)
        {0x30, {0x5B}}, // ] (Right Bracket)
        {0x31, {0x5D}}, // \ (Backslash)
        {0x32, {0x2F}}, // # (non-US keyboard hash)
        {0x33, {0x4C}}, // ; (Semicolon)
        {0x34, {0x52}}, // ' (Quote)
        {0x35, {0x0E}}, // ` (Grave)
        {0x36, {0x41}}, // , (Comma)
        {0x37, {0x49}}, // . (Period)
        {0x38, {0x4A}}, // / (Slash)
        {0x39, {0x58}}, // Caps Lock

        // F keys
        {0x3A, {0x05}}, // F1
        {0x3B, {0x06}}, // F2
        {0x3C, {0x04}}, // F3
        {0x3D, {0x0C}}, // F4
        {0x3E, {0x03}}, // F5
        {0x3F, {0x0B}}, // F6
        {0x40, {0x83}}, // F7
        {0x41, {0x0A}}, // F8
        {0x42, {0x01}}, // F9
        {0x43, {0x09}}, // F10
        {0x44, {0x78}}, // F11
        {0x45, {0x07}}, // F12

        {0x46, {0xE0, 0x12, 0xE0, 0x7C}}, // Print Screen
        {0x47, {0x7E}}, // Scroll Lock
        {0x48, {0xE1, 0x14, 0x77, 0xE1, 0xF0, 0x14, 0xE0, 0x77}}, // Pause/Break
        {0x49, {0xE0, 0x70}}, // Insert
        {0x4A, {0xE0, 0x6C}}, // Home
        {0x4B, {0xE0, 0x7D}}, // Page Up
        {0x4C, {0xE0, 0x71}}, // Delete
        {0x4D, {0xE0, 0x69}}, // End
        {0x4E, {0xE0, 0x7A}}, // Page Down
        {0x4F, {0xE0, 0x74}}, // Right Arrow
        {0x50, {0xE0, 0x6B}}, // Left Arrow
        {0x51, {0xE0, 0x72}}, // Down Arrow
        {0x52, {0xE0, 0x75}}, // Up Arrow
        
        {0x53, {0x77}}, // Keyboard Num Lock and Clear
        {0x54, {0xE0, 0x4A}}, // Keypad /
        {0x55, {0x7C}}, // Keypad *
        {0x56, {0x7B}}, // Keypad -
        {0x57, {0x79}}, // Keypad +
        {0x58, {0xE0, 0x5A}}, // Keypad Enter
        {0x59, {0x69}}, // Keypad 1
        {0x5A, {0x72}}, // Keypad 2
        {0x5B, {0x7A}}, // Keypad 3
        {0x5C, {0x6B}}, // Keypad 4
        {0x5D, {0x73}}, // Keypad 5
        {0x5E, {0x74}}, // Keypad 6
        {0x5F, {0x6C}}, // Keypad 7
        {0x60, {0x75}}, // Keypad 8
        {0x61, {0x7D}}, // Keypad 9
        {0x62, {0x70}}, // Keypad 0
        {0x63, {0x71}}, // Keypad .

        // {0x64, {0xE0, 0x78}}, // Keyboard non-us \ and |
        // {0x67, {0xE0, 0x77}}, // Keypad =

        {0xE0, {0x14}}, // Left Control
        {0xE4, {0xE0, 0x14}}, // Right Control
        {0xE1, {0x12}}, // Left Shift
        {0xE5, {0x59}}, // Right Shift
        {0xE2, {0x11}}, // Left Alt
        {0xE6, {0xE0, 0x11}}, // Right Alt
        {0xE3, {0xE0, 0x1F}}, // Left GUI (Windows)
        {0xE7, {0xE0, 0x27}}, // Right GUI
    };
}

extern "C" void overlay_printf(const char *fmt, ...);

ps2_scancode_t usb_to_ps2(uint8_t usbCode, bool is_break) {
    // Mapping of USB HID codes to PS/2 make code sequences
    ps2_scancode_t r;
 
    if (usbToPs2MakeMap.empty()) {
        init_usb_to_ps2_map();
    }
 
    r.len = 0;
    // pause does not have a break code
    if (usbCode == 0x48 && is_break) return r;

    auto it = usbToPs2MakeMap.find(usbCode);

    // overlay_printf("usb_to_ps2: found=%d\n", it != usbToPs2MakeMap.end());

    if (it != usbToPs2MakeMap.end()) {
        int len = it->second.size();
        r.len = is_break ? len + 1 : len;

        // overlay_printf("usb_to_ps2: len=%d\n", r.len);

        if (!is_break) {
            for (int i = 0; i < len; i++)
                r.code[i] = it->second[i];
        } else {
            if (len > 1) {
                r.code[0] = 0xE0;
                r.code[1] = 0xF0;
                for (int i = 1; i < len; i++)
                    r.code[i + 1] = it->second[i];
            } else {
                r.code[0] = 0xF0;
                r.code[1] = it->second[0];
            }
        }
    }
    return r;
}