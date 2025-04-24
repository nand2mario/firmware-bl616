#include <string.h>
#include "ff.h"

#include "utils.h"

bool floppy_mounted = false;
USB_NOCACHE_RAM_SECTION FIL ffloppy;

static IOWR(uint16_t addr, uint16_t data) {
    taskENTER_CRITICAL();
    bflb_uart_putchar(uart1_dev, 0x0a);
    bflb_uart_putchar(uart1_dev, addr >> 8);
    bflb_uart_putchar(uart1_dev, addr & 0xff);
    bflb_uart_putchar(uart1_dev, data >> 8);
    bflb_uart_putchar(uart1_dev, data & 0xff);
    taskEXIT_CRITICAL();
}

int loadpc(const char *fname) {
    FILINFO fno;

    DEBUG("loadpc start\n");
    // check extension .img
    char *p = strcasestr(fname, ".img");
    if (p == NULL) {
        overlay_message("Only .img floppy supported", 1);
        return -1;
    }

    if (!core_running) {
        const char *BIOS = "usb:pc/bios.bin";
        UINT br;

        // load BIOS
        if (f_stat(BIOS, &fno) != FR_OK) {
            overlay_message( "Cannot find /pc/bios.bin", 1);
            return -1;
        }
        if (f_open(&fcore, BIOS, FA_READ) != FR_OK) {
            overlay_message( "Cannot open /pc/bios.bin", 1);
            return -1;
        }
        set_loading_state(1);
        do {
            if (f_read(&fcore, fbuf, 1024, &br) != FR_OK)
                break;
            send_fbuf_data(br);
        } while (br == 1024);        
        f_close(&fcore);
    }

    // mount floppy image
    if (f_stat(fname, &fno) != FR_OK) {
        overlay_message( "Cannot find floppy image", 1);
        return -1;
    }
    int kb = fno.fsize / 1024;

    if (f_open(&ffloppy, fname, FA_READ | FA_WRITE) != FR_OK) {
        overlay_message( "Cannot open floppy image", 1);
        return -1;
    }
    uint16_t cylinders, sectors_per_track, total_sectors, heads;
    if (kb == 360) {
        cylinders = 40;
        sectors_per_track = 9;
        total_sectors = 720;
        heads = 2;
    } else if (kb == 180) {
        cylinders = 40;
        sectors_per_track = 9;
        total_sectors = 360;
        heads = 1;
    } else if (kb == 720) {
        cylinders = 80;
        sectors_per_track = 18;
        total_sectors = 1440;
        heads = 1;
    } else if (kb == 1440) {
        cylinders = 80;
        sectors_per_track = 18;
        total_sectors = 2880;
        heads = 2;
    } else {
        overlay_message( "Unsupported floppy image size", 1);
        return -1;
    }
    IOWR(0xf200, 1);    // media present
    IOWR(0xf201, 0);    // write protect
    IOWR(0xf202, cylinders);
    IOWR(0xf203, sectors_per_track);  
    IOWR(0xf204, total_sectors);
    IOWR(0xf205, heads);   

    floppy_mounted = true;

    if (!core_running) {
        set_loading_state(0);
        core_running = true;
    }

    return 0;
}