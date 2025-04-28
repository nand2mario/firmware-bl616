#define _GNU_SOURCE
#include <string.h>      // for strcasestr

#include "utils.h"
#include "cores.h"
#include "overlay.h"

int loadsms(const char *fname) {
    DEBUG("loadsms start\n");
    FRESULT r = FR_NO_FILE;

    // check extension .bin
    char *p = strcasestr(fname, ".sms");
    if (p == NULL) {
        overlay_message("Only .sms, .gg, and .sg supported", 1);
        return r;
    }

    r = f_open(&fcore, fname, FA_READ);
    if (r) {
        overlay_status("Cannot open file");
        return r;
    }
    unsigned int off = 0, br, total = 0;
    unsigned int size = get_file_size(fname);
    off = size % 1024;          // skipping 512-byte header if there is one

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

    DEBUG("loadsms: %d bytes\n", total);
    overlay_status("Success");
    core_running = true;

    overlay(0);		// turn off OSD

loadmd_close_file:
    set_loading_state(0);   // turn off game loading, this starts the core
    f_close(&fcore);
    return r;
}

