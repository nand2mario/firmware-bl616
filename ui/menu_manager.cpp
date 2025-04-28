#include "menu_manager.h"
#include <memory>
#include <vector>

class MenuManager {
private:
    std::vector<std::unique_ptr<Menu>> menu_stack;
    
public:
    void push_menu(std::unique_ptr<Menu> menu) {
        menu_stack.push_back(std::move(menu));
    }
    
    void pop_menu() {
        if (!menu_stack.empty()) {
            menu_stack.pop_back();
        }
    }
    
    Menu* menu_current() {
        if (menu_stack.empty()) return nullptr;
        return menu_stack.back().get();
    }
    
    void menu_render() {
        if (auto* menu = menu_current()) {
            menu->render();
        }
    }
    
    void menu_handle_input(int key) {
        if (auto* menu = menu_current()) {
            menu->handle_input(key);
        }
    }
};

static MenuManager menu_manager;

void push_menu(std::unique_ptr<Menu> menu) {
    menu_manager.push_menu(std::move(menu));
}

void pop_menu() {
    menu_manager.pop_menu();
}

Menu* menu_current() {
    return menu_manager.menu_current();
}

void menu_render() {
    menu_manager.menu_render();
}

void menu_handle_input(int key) {
    menu_manager.menu_handle_input(key);
} 