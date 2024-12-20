//
// Created by DUSK on 2024/12/17.
//

#include "ui.hpp"
#include "GUI.hpp"

auto Screen::init() -> void
{
    Component::set_parent(gui->main.screen);

    // 按钮初始化
    Button button;
    button.init(gui->main.btn_test,gui->main.btn_test_label,100,300,100,100,"");
}

auto Events::init() -> void
{
}
