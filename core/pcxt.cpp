#define _GNU_SOURCE
#include <string.h>      // for strcasestr

#include "menu_manager.h"
#include "utils.h"
#include "cores.h"
#include "overlay.h"
#include "file_chooser.h"

bool floppy[2];
std::string floppy_fname[2];
USB_NOCACHE_RAM_SECTION FIL f_floppy[2];

static void IOWR(uint16_t addr, uint16_t data) {
    taskENTER_CRITICAL();
    fpga_tx_header(0x0b, 5);
    fpga_tx_byte(addr >> 8);
    fpga_tx_byte(addr & 0xff);
    fpga_tx_byte(data >> 8);
    fpga_tx_byte(data & 0xff);
    taskEXIT_CRITICAL();
}

// open floppy image and set parameters to core
bool mount_floppy(int drive, const char *fname) {
    // close any open image
    if (floppy[drive]) {
        f_close(&f_floppy[drive]);
    }

    FILINFO fno;
    if (f_stat(fname, &fno) != FR_OK) {
        overlay_message( "Cannot stat floppy image", 1);
        return false;
    }
    int kb = fno.fsize / 1024;
    if (f_open(&f_floppy[drive], fname, FA_READ | FA_WRITE) != FR_OK) {
        overlay_message( "Cannot open floppy image", 1);
        return false;
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
    } else if (kb == 160) {
        cylinders = 40;
        sectors_per_track = 8;
        total_sectors = 320;
        heads = 1;
    } else if (kb == 320) {
        cylinders = 40;
        sectors_per_track = 8;
        total_sectors = 640;
        heads = 2;
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
        overlay_message( "Unsupported image size", 1);
        return -1;
    }
    DEBUG("set floppy parameters\n");
    uint16_t off = drive ? 0xf0 : 0x00;
    IOWR(0xf200 + off, 1);    // media present
    IOWR(0xf201 + off, 0);    // write protect
    IOWR(0xf202 + off, cylinders);
    IOWR(0xf203 + off, sectors_per_track);  
    IOWR(0xf204 + off, total_sectors);
    IOWR(0xf205 + off, heads);   

    floppy[drive] = true;
    std::string s = fname;
    if (s.find_last_of('/') != std::string::npos)
        floppy_fname[drive] = s.substr(s.find_last_of('/') + 1);
    else
        floppy_fname[drive] = s;
    return true;
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
        DEBUG("load BIOS\n");
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
        DEBUG("load BIOS done\n");
    }

    // mount floppy image
    mount_floppy(0, fname);

    if (!core_running) {
        set_loading_state(0);
        core_running = true;
    }

    overlay(0);

    return 0;
}

// PcxtMenu implementation
PcxtMenu::PcxtMenu(const char *imgdir) : imgdir(imgdir) {}

void PcxtMenu::render() {
    overlay_clear();
    overlay_cursor(0, 10);
    //              012345678901234567890123456789012
    overlay_printf("          --- PCXT ---          \n");
    overlay_cursor(0, 12);
    if (floppy[0])
        overlay_printf("  A: %s\n", floppy_fname[0].c_str());
    else
        overlay_printf("  A: <empty>\n");
    overlay_cursor(0, 13);
    if (floppy[1])
        overlay_printf("  B: %s\n", floppy_fname[1].c_str());
    else
        overlay_printf("  B: <empty>\n");
    overlay_cursor(0, 15);
    overlay_printf("  Reset Core\n");
    overlay_cursor(0, 17);
    overlay_printf("  << Main Menu\n");
}

std::vector<int> PcxtMenu::get_options() {
    return {12, 13, 15, 17};
}

bool PcxtMenu::on_choose(int idx) {
    if (idx == 0 || idx == 1) {
        delay(200);
        overlay_printf("Dir: %s\n", imgdir);
        FileChooser c;
        c.rootdir = imgdir;
        c.curdir = imgdir;
        c.msg_return = "<< Cancel";
        std::string fname;
        c.choose_file(fname);
        delay(200);
        if (!fname.empty()) {
            mount_floppy(idx, fname.c_str());
            do_redraw();
            return false;       // don't close menu
        }
    } else if (idx == 2) {
        overlay_printf("Reset Core\n");
        set_loading_state(1);
        set_loading_state(0);
        return true;            // close menu
    } else if (idx == 3) {
        overlay_printf("<< Main Menu\n");
        return true;
    }
    return false;
}

