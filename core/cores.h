#ifndef CORES_H
#define CORES_H

#include <vector>
#include <string>

struct core_info {
    uint16_t id;                    // 1: NES, 2: SNES, 3: GB, 4: GENESIS, 0: end
    const char *display_name;
    const char *rom_dir;            // usb:nes, usb:snes, etc.
    const char *core_file;          // core file in cores/
    int (*load_rom)(const char *fname);
};

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

#endif 