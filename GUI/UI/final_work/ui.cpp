//
// Created by DUSK on 2024/12/17.
//

#include "ui.hpp"
#include "GUI.hpp"
#include "beep.h"
auto Screen::init() -> void
{
    Component::set_parent(gui->main.screen);

    // 按钮初始化
    Button button;
    button.init(gui->main.btn_test,gui->main.btn_test_label,100,300,100,100,"");
}

auto Events::init() -> void
{
// lambda匿名

    Events::bond(gui->main.btn_test,btn_fun(
            []()
            {
                static bool flag = false;
                flag = !flag;
                if(flag)
                {
                    beep_start();
                }
                else
                {
                    beep_stop();
                }
            }
            ));
}
