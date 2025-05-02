#define _GNU_SOURCE
#include <string.h>      // for strcasestr

#include "utils.h"
#include "cores.h"
#include "overlay.h"
// return 0 if snes header is successfully parsed at off
// typ 0: LoROM, 1: HiROM, 2: ExHiROM
int parse_snes_header(FIL *fp, int pos, int file_size, int typ, unsigned char *hdr,
                      int *map_ctrl, int *rom_type_header, int *rom_size,
                      int *ram_size, int *company) {
    unsigned int br;
    if (f_lseek(fp, pos))
        return 1;
    f_read(fp, hdr, 64, &br);
    if (br != 64) return 1;
    int mc = hdr[21];
    int rom = hdr[23];
    int ram = hdr[24];
    int checksum = (hdr[28] << 8) + hdr[29];
    int checksum_compliment = (hdr[30] << 8) + hdr[31];
    int reset = (hdr[61] << 8) + hdr[60];
    int size2 = 1024 << rom;

    overlay_status("size=%d", size2);

    // calc heuristics score
    int score = 0;		
    if (size2 >= file_size) score++;
    if (rom == 1) score++;
    if (checksum + checksum_compliment == 0xffff) score++;
    int all_ascii = 1;
    for (int i = 0; i < 21; i++)
        if (hdr[i] < 32 || hdr[i] > 127)
            all_ascii = 0;
    score += all_ascii;

    overlay_status("pos=%x, type=%d, map_ctrl=%d, rom=%d, ram=%d, checksum=%x, checksum_comp=%x, reset=%x, score=%d\n", 
            pos, typ, mc, rom, ram, checksum, checksum_compliment, reset, score);

    if (rom < 14 && ram <= 7 && score >= 1 && 
        reset >= 0x8000 &&				// reset vector position correct
       ((typ == 0 && (mc & 3) == 0) || 	// normal LoROM
        (typ == 0 && mc == 0x53)    ||	// contra 3 has 0x53 and LoROM
        (typ == 1 && (mc & 3) == 1) ||	// HiROM
        (typ == 2 && (mc & 3) == 2))) {	// ExHiROM
        *map_ctrl = mc;
        *rom_type_header = hdr[22];
        *rom_size = rom;
        *ram_size = ram;
        *company = hdr[26];
        return 0;
    }
    return 1;
}

// TODO: implement bsram backup
// return 0 if successful
int loadsnes(const char *fname) {
    int r = 1;
    DEBUG("loadsnes start");

    // check extension .sfc or .smc
    char *p = strcasestr(fname, ".sfc");
    if (p == NULL)
        p = strcasestr(fname, ".smc");
    if (p == NULL) {
        overlay_message("Only .smc or .sfc supported", 1);
        return r;
    }

    r = f_open(&fcore, fname, FA_READ);
    if (r) {
        overlay_status("Cannot open file");
        return r;
    }
    unsigned int br, total = 0;
    int size = get_file_size(fname);
    int map_ctrl, rom_type_header, rom_size, ram_size, company;
    // parse SNES header from ROM file
    int off = size & 0x3ff;		// rom header (0 or 512)
    int header_pos;
    overlay_status("snes rom header offset: %d\n", off);
    
    header_pos = 0x7fc0 + off;
    if (parse_snes_header(&fcore, header_pos, size-off, 0, fbuf, &map_ctrl, &rom_type_header, &rom_size, &ram_size, &company)) {
        header_pos = 0xffc0 + off;
        if (parse_snes_header(&fcore, header_pos, size-off, 1, fbuf, &map_ctrl, &rom_type_header, &rom_size, &ram_size, &company)) {
            header_pos = 0x40ffc0 + off;
            if (parse_snes_header(&fcore, header_pos, size-off, 2, fbuf, &map_ctrl, &rom_type_header, &rom_size, &ram_size, &company)) {
                overlay_status("Not a SNES ROM file");
                delay(200);
                goto loadsnes_close_file;
            }
        }
    }

    // load actual ROM
    set_loading_state(1);		// enable game loading, this resets SNES
    core_running = false;

    // Send 64-byte header to snes
    send_fbuf_data(64);

    // Send rom content to snes
    if ((r = f_lseek(&fcore, off)) != FR_OK) {
        overlay_status("Seek failure");
        goto loadsnes_snes_end;
    }
    do {
        if ((r = f_read(&fcore, fbuf, 1024, &br)) != FR_OK)
            break;
        if (br == 0) break;
        send_fbuf_data(br);
        total += br;
        if ((total & 0xffff) == 0) {	// display progress every 64KB
            overlay_status("%d/%dK", total >> 10, size >> 10);
            if ((map_ctrl & 3) == 0)
                overlay_printf(" Lo");
            else if ((map_ctrl & 3) == 1)
                overlay_printf(" Hi");
            else if ((map_ctrl & 3) == 2)
                overlay_printf(" ExHi");
            //              01234567890123456789012345678901
            overlay_printf(" ROM=%d RAM=%d                 ", 1 << rom_size, ram_size ? (1 << ram_size) : 0);
        }
    } while (br == 1024);

    overlay_status("Success");
    core_running = true;

    overlay(0);		// turn off OSD

loadsnes_snes_end:
    set_loading_state(0);	// turn off game loading, this starts SNES
loadsnes_close_file:
    f_close(&fcore);
    return r;
}
