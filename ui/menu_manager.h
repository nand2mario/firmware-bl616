#ifndef MENU_MANAGER_H
#define MENU_MANAGER_H

#include <memory>

class Menu {
public:
    virtual ~Menu() = default;
    virtual void render() = 0;
    virtual void handle_input(int key) = 0;
};

void push_menu(std::unique_ptr<Menu> menu);
void pop_menu();
Menu* menu_current();
void menu_render();
void menu_handle_input(int key);

#endif // MENU_MANAGER_H 