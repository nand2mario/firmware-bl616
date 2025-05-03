#define _GNU_SOURCE
#include <string.h>      // for strcasestr

#include "utils.h"
#include "cores.h"
#include "overlay.h"

// Core specific state
bool gba_bios_loaded;
bool gba_missing_bios_warned;


// check if gba_bios.bin is present in the root directory
// if not, warn user, if present, load it
void gba_load_bios() {
    if (gba_bios_loaded | gba_missing_bios_warned) return;

    DEBUG("gba_load_bios start\n");
    FILINFO fno;
    std::string bios_path(drv);
    bios_path.append("gba/gba_bios.bin");
    if (f_stat(bios_path.c_str(), &fno) != FR_OK) {
        overlay_message( "Cannot find gba_bios.bin\n"
                 "Using open source BIOS\n"
                 "Expect low compatibility", 1);
        gba_missing_bios_warned = 1;
        return;
    }

    int r = 1;
    unsigned br;
    if (f_open(&fcore, bios_path.c_str(), FA_READ) != FR_OK) {
        overlay_message("Cannot open /gba/gba_bios.bin", 1);
        return;
    }
    set_loading_state(4);
    do {
        if ((r = f_read(&fcore, fbuf, 1024, &br)) != FR_OK)
            break;
        send_fbuf_data(br);
    } while (br == 1024);

    f_close(&fcore);
    gba_bios_loaded = 1;
    DEBUG("gba_load_bios end\n");
}

int loadgba(const char *fname) {
    DEBUG("loadgba start\n");
    FRESULT r = FR_NO_FILE;

    // check extension .gba
    char *p = strcasestr(fname, ".gba");
    if (p == NULL) {
        overlay_message("Only .gba supported", 1);
        return r;
    }

    unsigned int size = get_file_size(fname);

    r = f_open(&fcore, fname, FA_READ);
    if (r) {
        overlay_status("Cannot open file");
        return r;
    }
    unsigned int off = 0, br, total = 0;

    // load actual ROM
    set_loading_state(1);		// enable game loading, this resets GBA
    core_running = false;

    // Send rom content to gba
    if ((r = f_lseek(&fcore, off)) != FR_OK) {
        overlay_status("Seek failure");
        goto loadgba_close;
    }
    // int detect = 0; // 1: past 'EEPR', 2: past 'FLAS', 3: past 'SRAM'
    // gba_backup_type = GBA_BACKUP_NONE;
    do {
        if ((r = f_read(&fcore, fbuf, 1024, &br)) != FR_OK)
            break;

        send_fbuf_data(br);
        // TODO: do backup type detection

        total += br;
        if ((total & 0xffff) == 0) {	// display progress every 64KB
            //              01234567890123456789012345678901
            overlay_status("%d/%dK                          ", total >> 10, size >> 10);
        }
    } while (br == 1024);

    DEBUG("loadgba: %d bytes rom sent.\n", total); 

    gba_load_bios();

    overlay_status("Success");
    core_running = true;

    overlay(0);		// turn off OSD

loadgba_close:
    set_loading_state(0);   // turn off game loading, this starts the core
    f_close(&fcore);
    return r;
}
