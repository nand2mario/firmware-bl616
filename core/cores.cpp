#include <vector>
#include <string>
#include "cores.h"

#include "ff.h"
#include "menu_manager.h"
#include "overlay.h"

// null-terminated list of core info
std::vector<core_info> core_info_list;
// Main menu listing:
// >0: core id, -1: cores menu, -2: options menu, 0: end of list
std::vector<int16_t> main_menu_config;

struct core_info *find_core_by_id(uint16_t id) {
    for (auto &core : core_info_list) {
        if (core.id == id)
            return &core;
    }
    return NULL;
}

Menu *create_default_menu(const char *imgdir) {
    dprint("Creating default menu\n");
    return new DefaultMenu();
}

Menu *create_pcxt_menu(const char *imgdir) {
    dprint("Creating PCXT menu\n");
    return new PcxtMenu(imgdir);
}

void init_core_list() {
    core_info_list = {
        {1, "NES", "nes", "nestang.bin", loadnes, create_default_menu},
        {2, "SNES", "snes", "snestang.bin", loadsnes, create_default_menu},
        {3, "Game Boy Advance", "gba", "gbatang.bin", loadgba, create_default_menu},
        {4, "MegaDrive / Genesis", "genesis", "mdtang.bin", loadmd, create_default_menu},
        {5, "Sega Master System", "sms", "smstang.bin", loadsms, create_default_menu},
        {6, "IBM PC/XT", "pc", "pctang.bin", loadpc, create_pcxt_menu}
    };

    main_menu_config = {1,2,
#if defined(TANG_MEGA60K) || defined(TANG_MEGA138K) || defined(TANG_CONSOLE60K) || defined(TANG_CONSOLE138K)
        3,4,5,6,
#endif
        -1, -2
    };
}


extern const char *BOARD_NAME;
extern char *drv;

// Find a core file in the search order:
// usb:cores/${BOARD_NAME}/${core_name}
// usb:cores/${core_name}
bool find_core_for_board(std::string &fname, const char *core_name) {
    // check sd|usb:cores/${BOARD_NAME}/${core_name}
    fname = std::string(drv) + "cores/" + BOARD_NAME + "/" + core_name;
    FILINFO fno;
    if (f_stat(fname.c_str(), &fno) == FR_OK && fno.fsize > 0) {
        return true;
    }

    // check sd|usb:cores/${core_name}
    fname = std::string(drv) + "cores/" + core_name;
    if (f_stat(fname.c_str(), &fno) == FR_OK && fno.fsize > 0) {
        return true;
    }
    return false;
}
