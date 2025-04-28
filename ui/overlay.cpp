#include <string>
#include <stdarg.h>
#include <algorithm>

extern "C" {
#include <FreeRTOS.h>
#include <task.h>
}
#include "utils.h"
#include "overlay.h"

/////////////////////////////////////////////////////////////////////////////////
// Overlay and other core control over UART


int _overlay_on = 1;
int overlay_on() {
    return _overlay_on;
}

void overlay_cursor(int col, int row) {
    // uart1 command: 4 x[7:0] y[7:0]
    taskENTER_CRITICAL();
    fpga_tx_header(0x04, 3);
    fpga_tx_byte(col);
    fpga_tx_byte(row);
    taskEXIT_CRITICAL();
}

// print to UART without the core displaying it. liveuart.py catches this.
void dprint(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    char buf[256];
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    taskENTER_CRITICAL();
    int len = strlen(buf);
    fpga_tx_header(0x0d, len+1);
    for(int i = 0; i < len; i++) {
        fpga_tx_byte(buf[i]);
    }
    taskEXIT_CRITICAL();
}

void overlay_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    char buf[256];
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    taskENTER_CRITICAL();
    int len = strlen(buf);
    fpga_tx_header(0x05, len+1);
    for(int i = 0; i < len; i++) {
        fpga_tx_byte(buf[i]);
    }
    taskEXIT_CRITICAL();
}

void overlay_clear() {
    for (int i = 0; i < 28; i++) {
        overlay_cursor(0, i);
        //              01234567890123456789012345678901
        overlay_printf("                                ");
    }
}

void overlay_status(const char *fmt, ...) {
    overlay_cursor(1, 27);
    va_list args;
    va_start(args, fmt);
    // char buf[256];
    overlay_printf(fmt, args);
    //vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
}

// show a pop-up message, press any key to discard (caller needs to redraw screen)
// msg: could be multi-line (separate with \n), max 10 lines
// center: whether to center the text
void overlay_message(const char *msg, int center) {
    // count number of lines and max width
    int w[10], lines=10, maxw = 0;
    int len = strlen(msg);
    const char *end = msg + len;
    const char *sol = msg;
    for (int i = 0; i < 10; i++) {
        const char *eol = strchr(sol, '\n');
        if (eol) { // found \n
            w[i] = std::min(eol - sol, 26);
            maxw = std::max(w[i], maxw);
            sol = eol+1;
        } else {
            w[i] = std::min(end - sol, 26);
            maxw = std::max(w[i], maxw);
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
    const char *s = msg;
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

// turn overlay on/off
void overlay(int state) {
    taskENTER_CRITICAL();
    _overlay_on = state;
    fpga_tx_header(0x08, 2);
    fpga_tx_byte(state);        
    taskEXIT_CRITICAL();
}

