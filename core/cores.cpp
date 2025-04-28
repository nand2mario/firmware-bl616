#include <vector>
#include <string>
#include "ff.h"
#include "cores.h"

// null-terminated list of core info
std::vector<core_info> core_info_list;
// Main menu listing:
// >0: core id, -1: cores menu, -2: options menu, 0: end of list
std::vector<int16_t> main_menu_config;

void init_core_list() {
    core_info_list = {
        {1, "NES", "usb:nes", "nestang.bin", loadnes},
        {2, "SNES", "usb:snes", "snestang.bin", loadsnes},
        {3, "Game Boy Advance", "usb:gba", "gbatang.bin", loadgba},
        {4, "MegaDrive / Genesis", "usb:genesis", "mdtang.bin", loadmd},
        {5, "Sega Master System", "usb:sms", "smstang.bin", loadsms},
        {6, "IBM PC/XT", "sd:pc", "pctang.bin", loadpc}
    };

    main_menu_config = {1,2,
#if defined(TANG_MEGA60K) || defined(TANG_MEGA138K) || defined(TANG_CONSOLE60K) || defined(TANG_CONSOLE138K)
        3,4,5,6,
#endif
        -1, -2
    };
}

extern const char *BOARD_NAME;

// Find a core file in the search order:
// usb:cores/${BOARD_NAME}/${core_name}
// usb:cores/${core_name}
bool find_core_for_board(std::string &fname, const char *core_name) {
    // check usb:cores/${BOARD_NAME}/${core_name}
    fname = std::string("usb:cores/") + BOARD_NAME + "/" + core_name;
    FILINFO fno;
    if (f_stat(fname.c_str(), &fno) == FR_OK && fno.fsize > 0) {
        return true;
    }

    // check usb:cores/${core_name}
    fname = std::string("usb:cores/") + core_name;
    if (f_stat(fname.c_str(), &fno) == FR_OK && fno.fsize > 0) {
        return true;
    }
    return false;
}
