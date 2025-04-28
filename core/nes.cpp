#define _GNU_SOURCE
#include <string.h>      // for strcasestr

#include "utils.h"
#include "cores.h"
#include "overlay.h"

// Load a NES ROM
// return 0 if successful
int loadnes(const char *fname) {
    unsigned int off = 0, br, total = 0;
    unsigned int size;
    int r = 1;
    DEBUG("loadnes start\n");

    // check extension .nes
    char *p = strcasestr(fname, ".nes");
    if (p == NULL) {
        overlay_message("Only .nes supported", 1);
        goto loadnes_end;
    }

    r = f_open(&fcore, fname, FA_READ);
    if (r) {
        overlay_status("Cannot open file");
        goto loadnes_end;
    }
    size = get_file_size(fname);

    // load actual ROM
    set_loading_state(1);
    core_running = false;

    // Send rom content
    if ((r = f_lseek(&fcore, off)) != FR_OK) {
        overlay_status("Seek failure");
        goto loadnes_snes_end;
    }


    do {
        if ((r = f_read(&fcore, fbuf, 1024 /*BLOCK_SIZE*/, &br)) != FR_OK)
            break;
        // start rom loading command
        send_fbuf_data(br);
        taskYIELD();                // allow gamepad polling to run
        total += br;
        if ((total & 0xfff) == 0) {	// display progress every 4KB
            //              01234567890123456789012345678901
            overlay_status("%d/%dK                          ", total >> 10, size >> 10);
        }
    } while (br == 1024 /*BLOCK_SIZE*/);

    DEBUG("loadnes: %d bytes\n", total);
    overlay_status("Success");
    core_running = true;

    overlay(0);		// turn off OSD

loadnes_snes_end:
    set_loading_state(0);   // turn off game loading, this starts the core
    f_close(&fcore);
loadnes_end:
    return r;
}
