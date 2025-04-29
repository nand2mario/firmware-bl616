#include "menu_manager.h"
#include <memory>
#include <vector>
#include "utils.h"

std::vector<std::unique_ptr<Menu>> menu_stack;

void menu_clear() {
    DEBUG("Menu clear\n");
    menu_stack = std::vector<std::unique_ptr<Menu>>();
    DEBUG("Menu clear done\n");
}

void push_menu(std::unique_ptr<Menu> menu) {
    menu_stack.push_back(std::move(menu));
}

void pop_menu() {
    menu_stack.pop_back();
}

Menu* menu_current() {
    return menu_stack.back().get();
}

bool menu_is_active() {
    return !menu_stack.empty();
} 

void menu_input_loop() {
    if (!menu_is_active())
        return;

    DEBUG("Menu input loop\n");
    std::vector<int> options = menu_current()->get_options();
    int last = 0, active = 0;
    while (menu_is_active() && overlay_on()) {
        if (menu_current()->redraw) {
            menu_current()->render();
            menu_current()->redraw = false;
        }

        uint16_t joy1=0, joy2=0, hid1=0, hid2=0;    
        get_joypad_states(&joy1, &joy2, &hid1, &hid2);
        joy1 |= hid1;
        joy2 |= hid2;

        // DEBUG("Joy1: %04x, Joy2: %04x\n", joy1, joy2);

        if (joy1 == OSD_KEY_CODE || joy2 == OSD_KEY_CODE) {
            overlay(0);    // turn off OSD
            delay(300);
        }

        if ((joy1 & 0x10) || (joy2 & 0x10)) {   // up
            if (active > 0) active--;
        }
        if ((joy1 & 0x20) || (joy2 & 0x20)) {   // down
            if (active < options.size()-1) active++;
        }

        for (int i = 0; i < options.size(); i++) {
            overlay_cursor(0, options[i]);
            if (i == active)
                overlay_printf(">");
            else
                overlay_printf(" ");
        }

        if ((joy1 & 0x100) || (joy2 & 0x100) || // button A pressed
            (joy1 & 0x1) || (joy2 & 0x1))       // button B pressed
        {
            int depth = menu_stack.size();
            bool r = menu_current()->on_choose(active);
            if (r) {
                pop_menu();
                if (!menu_is_active())
                    return;
            }
            if (menu_stack.size() != depth) {   // menu changed
                options = menu_current()->get_options();
                if (active >= options.size())
                    active = options.size()-1;
                menu_current()->do_redraw();
            }
        }
        if (last != active)
            delay(100);
        last = active;
    }
}
