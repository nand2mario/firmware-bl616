#pragma once

#include <vector>
#include <string>
#include "menu_manager.h"
#include "usb_config.h"
#include "ff.h"

struct core_info {
    uint16_t id;                    // 1: NES, 2: SNES, 3: GB, 4: GENESIS, 0: end
    const char *display_name;
    const char *rom_dir;            // nes, snes, etc.
    const char *core_file;          // core file in cores/
    int (*load_rom)(const char *fname);
    Menu *(*create_menu)(const char *imgdir);
};

extern struct core_info *find_core_by_id(uint16_t id);

extern std::vector<core_info> core_info_list;
extern std::vector<int16_t> main_menu_config;
extern void init_core_list();

extern int loadnes(const char *fname);
extern int loadsnes(const char *fname);
extern int loadgba(const char *fname);
extern int loadmd(const char *fname);
extern int loadsms(const char *fname);
extern int loadpc(const char *fname);

extern bool find_core_for_board(std::string &fname, const char *core_name);

struct PcxtMenu: Menu {
    const char *imgdir;
    PcxtMenu(const char *imgdir);
    virtual void render() override;
    virtual std::vector<int> get_options() override;
    virtual bool on_choose(int idx) override;
};

Menu *create_default_menu(const char *imgdir);
Menu *create_pcxt_menu(const char *imgdir);

extern bool floppy[2];
extern std::string floppy_fname[2];
extern USB_NOCACHE_RAM_SECTION FIL f_floppy[2];