/*
 * Copyright (c) 2026 Romain Tisserand
 * SPDX-License-Identifier: Apache-2.0
 */

// The "Options" menu. Cores already describe their
// options the MiSTer way in iosys_bl616.v's CONF_STR, e.g.
//     Tangcores;-;O12,OSD key,Right+Select,Select+Start,Select+RB;O3,Multitap,Off,On;-;V,v...
// where "O3" is one bit (bit 3) and "O12" a two-bit field (bits 1..2). This menu lists
// those entries, cycles a value on each press, and sends the whole word with command 0x03.

#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

#include "options_menu.h"
#include "utils.h"
#include "cores.h"
#include "ff.h"

extern const char *drv;

namespace {

struct CoreOption {
    std::string name;
    int lo, hi;                             // bit range in core_config
    std::vector<std::string> choices;
};

// MiSTer encodes bit numbers as one character each: 0-9 then A-V for 10-31.
int conf_bit(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'V') return c - 'A' + 10;
    return -1;
}

std::vector<std::string> split(const std::string &s, char sep) {
    std::vector<std::string> out;
    size_t start = 0;
    for (;;) {
        size_t p = s.find(sep, start);
        out.push_back(s.substr(start, p == std::string::npos ? std::string::npos : p - start));
        if (p == std::string::npos) break;
        start = p + 1;
    }
    return out;
}

std::vector<CoreOption> parse_conf_string(const std::string &conf) {
    std::vector<CoreOption> opts;
    for (const std::string &entry : split(conf, ';')) {
        if (entry.size() < 2 || entry[0] != 'O') continue;      // only plain 'O' options
        std::vector<std::string> f = split(entry, ',');
        if (f.size() < 3) continue;
        const std::string &bits = f[0];
        int lo = conf_bit(bits[1]);
        int hi = bits.size() > 2 ? conf_bit(bits[2]) : lo;
        if (lo < 0 || hi < lo) continue;
        // Declared by every TangCore core, but this firmware's OSD key is fixed
        // (OSD_KEY_CODE); listing it would offer a setting that does nothing.
        if (f[1] == "OSD key") continue;
        CoreOption o{f[1], lo, hi, {}};
        for (size_t i = 2; i < f.size(); i++) o.choices.push_back(f[i]);
        opts.push_back(o);
    }
    return opts;
}

// sd:cores/<core bitstream name>.opt holds the saved core_config word in hex.
std::string options_path(int core_id) {
    core_info *core = find_core_by_id(core_id);
    if (!core) return "";
    std::string name = core->core_file;
    size_t dot = name.rfind('.');
    if (dot != std::string::npos) name.resize(dot);
    return std::string(drv) + "cores/" + name + ".opt";
}

uint32_t load_options(int core_id) {
    std::string path = options_path(core_id);
    FIL f;
    char buf[16] = {0};
    UINT n = 0;
    if (path.empty() || f_open(&f, path.c_str(), FA_READ) != FR_OK) return 0;
    f_read(&f, buf, sizeof(buf) - 1, &n);
    f_close(&f);
    return (uint32_t)strtoul(buf, NULL, 16);
}

void save_options(int core_id, uint32_t value) {
    std::string path = options_path(core_id);
    FIL f;
    UINT n = 0;
    char buf[16];
    if (path.empty() || f_open(&f, path.c_str(), FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) return;
    int len = snprintf(buf, sizeof(buf), "%08lx\n", (unsigned long)value);
    f_write(&f, buf, len, &n);
    f_close(&f);
}

struct OptionsMenu : Menu {
    static const int TOP = 4;               // first option row
    int core_id;
    std::string title;
    std::vector<CoreOption> opts;
    uint32_t value = 0;
    bool have_conf = false;

    explicit OptionsMenu(int id) : core_id(id) {
        core_info *core = id > 0 ? find_core_by_id(id) : NULL;
        if (!core) return;
        title = core->display_name;
        std::string conf;
        have_conf = get_core_conf_string(conf);
        if (have_conf) opts = parse_conf_string(conf);
        value = load_options(id);
    }

    int field(const CoreOption &o) const {
        uint32_t mask = (1u << (o.hi - o.lo + 1)) - 1;
        return (value >> o.lo) & mask;
    }

    void render() override {
        overlay_clear();
        overlay_cursor(2, 1);
        overlay_printf("Options: %s", title.empty() ? "-" : title.c_str());
        int row = TOP;
        if (title.empty()) {
            overlay_cursor(2, row++);
            overlay_printf("Load a game first.");
        } else if (opts.empty()) {
            overlay_cursor(2, row++);
            overlay_printf(have_conf ? "This core has no options." : "Core did not answer.");
        }
        for (const CoreOption &o : opts) {
            int v = field(o);
            overlay_cursor(2, row++);
            overlay_printf("%s: %s", o.name.c_str(),
                           v < (int)o.choices.size() ? o.choices[v].c_str() : "?");
        }
        row++;
        overlay_cursor(2, row);
        overlay_printf("<< Back");
    }

    std::vector<int> get_options() override {
        std::vector<int> rows;
        int row = TOP + (opts.empty() ? 1 : 0);
        for (size_t i = 0; i < opts.size(); i++) rows.push_back(row++);
        rows.push_back(row + 1);            // "<< Back"
        return rows;
    }

    bool on_choose(int idx) override {
        if (idx >= (int)opts.size()) return true;       // back
        const CoreOption &o = opts[idx];
        uint32_t mask = (1u << (o.hi - o.lo + 1)) - 1;
        int n = o.choices.empty() ? 1 : (int)o.choices.size();
        uint32_t v = (field(o) + 1) % n;
        value = (value & ~(mask << o.lo)) | (v << o.lo);
        set_core_config(value);
        save_options(core_id, value);
        do_redraw();
        delay(200);                         // one press, one step
        return false;
    }
};

} // namespace

Menu *create_options_menu(int core_id) {
    return new OptionsMenu(core_id);
}

void apply_saved_core_options(int core_id) {
    set_core_config(load_options(core_id));
}
