#pragma once

#include <memory>
#include <vector>
#include "overlay.h"

// In core popup menu

struct Menu {
    bool redraw = false;
    virtual void do_redraw() {
        redraw = true;
    }
    virtual ~Menu() = default;
    virtual void render() = 0;
    virtual std::vector<int> get_options() = 0;   // return list of choice rows
    virtual bool on_choose(int idx) = 0;          // called when a choice is selected
                                                  // return true if menu should be popped
};

void menu_clear();
void push_menu(std::unique_ptr<Menu> menu);
void pop_menu();
Menu* menu_current();
void menu_render();
bool menu_is_active();     // true if the menu stack is not empty
void menu_input_loop();    // return when OSD is turned off or "<< Main Menu" is selected

struct DefaultMenu: Menu {
    DefaultMenu() {
    }

    void render() override {
        overlay_clear();
        overlay_cursor(0, 13);
        overlay_printf("<< Main Menu\n");
    }

    std::vector<int> get_options() override {
        return {13};
    }

    bool on_choose(int idx) override {
        return true;
    }
};

