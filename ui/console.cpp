#include "console.h"
#include "utils.h"
#include <vector>
#include <string>

#include "overlay.h"

// a simple console, mostly for debugging

struct console_s {
    std::vector<uint8_t> key_q;
    std::string command_buf[28];
    int command_row;

    uint8_t key_input() {
        if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
            if (key_buf[0] != 0 || key_buf[1] != 0 || key_buf[2] != 0 || key_buf[3] != 0) {
                DEBUG("key_input: key_buf={%d %d %d %d}\n", key_buf[0], key_buf[1], key_buf[2], key_buf[3]);
                for (int i = 0; i < 4; i++) {
                    if (key_buf[i] != 0) {
                        uint8_t key = key_buf[i];
                        key_q.push_back(key);
                        key_buf[i] = 0;
                    }
                }
            }
            xSemaphoreGive(state_mutex);
        }
        if (key_q.empty())
            return 0;
        uint8_t key = key_q[0];
        key_q.erase(key_q.begin());
        DEBUG("key_input: %d\n", key);
        return key;
    }

    void command_scroll(void) {
        // scroll screen
        for (int i = 0; i < 28; i++) {
            command_buf[i] = i < 27 ? command_buf[i+1] : std::string(32, ' ');
            overlay_cursor(0, i);
            overlay_printf(command_buf[i].c_str());
        }
        if (command_row > 0)
            command_row--;
    }

    std::string input_line() {
        std::string line;
        int pos = 1;
        while (command_row >= 28)
            command_scroll();
        bool caret = true;
        overlay_cursor(0, command_row);
        overlay_printf(">");
        while (1) {
            overlay_cursor(pos, command_row);
            if (caret) {
                overlay_printf("_");    // print caret
                caret = false;
            }
            uint8_t key = key_input();
            if (key > 0) {
                if (key == '\b') {
                    if (line.length() > 0)
                        line.pop_back();
                } else if (key == '\r' || key == '\n') {
                    return line;
                }else {
                    overlay_cursor(pos, command_row);
                    overlay_printf("%c", key);
                    line.push_back(key);
                }
                pos = line.length() < 31 ? line.length() + 1 : 31;
                caret = true;
            }
            delay(50);
        }
    }

    // print a line to command buffer, scroll screen if needed
    void command_print(std::string s) {
        while (command_row >= 28) 
            command_scroll();
        if (s.length() > 32)
            s = s.substr(0, 32);
        else if (s.length() < 32)
            s = s + std::string(32 - s.length(), ' ');
        command_buf[command_row] = s;
        overlay_cursor(0, command_row);
        overlay_printf(s.c_str());
        command_row++;
    }

    std::string tohex(uint32_t x) {
        std::string s(8, ' ');
        for (int i = 0; i < 8; i++) {
            s[7-i] = ("0123456789abcdef"[x & 0xf]);
            x >>= 4;
        }
        return s;
    }

    std::string trim(std::string x) {
        x.erase(0, x.find_first_not_of(" \t\n\r\f\v"));
        x.erase(x.find_last_not_of(" \t\n\r\f\v") + 1);
        return x;
    }

    void command_console(void) {
        overlay_clear();
        command_row = 0;

        while (1) {
            std::string line = trim(input_line());
            command_print('>'+line);
            if (line == "exit")
                break;
            else if (line == "help") {
                //             01234567890123456789012345678901
                command_print(" getconf    get core config");
                command_print(" setconf x  set core config");
            }
            else if (line == "getconf") {
                command_print(" core_id: "+std::to_string(get_core_id()));
                command_print(" core_config: "+tohex(get_core_config()));
            } else if (line.find("setconf ") == 0) {
                uint32_t x = std::stoul(line.substr(8), nullptr, 16);
                command_print(" set core config to "+tohex(x));
                set_core_config(x);
            } else if (line == "clear") {
                overlay_clear();
                command_row = 0;
            }
        }
    }
};

static console_s console;

uint8_t key_input(void) {
    return console.key_input();
}

void command_console(void) {
    console.command_console();
}


