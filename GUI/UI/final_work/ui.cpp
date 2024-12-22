//
// Created by DUSK on 2024/12/17.
//

#include "ui.hpp"
#include "GUI.hpp"
#include "beep.h"
#include "led.h"

auto Screen::init() -> void
{
    Component::set_parent(gui->main.screen);

    // 按钮初始化
    Button button;
    button.init(gui->main.btn_test, gui->main.btn_test_label, 100, 300, 100, 100, "蜂鸣器");

    button.init(gui->main.btn_led, gui->main.btn_led_label, 250, 300, 100, 100, "灯光");

}

auto Events::init() -> void
{
// lambda匿名

    bond(gui->main.btn_test, btn_fun(
            []() {
                static bool flag = false;
                flag = !flag;
                if (flag) {
                    beep_start();
                } else {
                    beep_stop();
                }
            }
    ));


    bond(gui->main.btn_led, [](event e) {
             static volatile bool flag2 = false;
             switch (lv_event_get_code(e)) {
                 case LV_EVENT_CLICKED:
                     flag2 = !flag2;
                     if (flag2) {
                         led_start();
                     } else {
                         led_stop();
                     }

                     break;
                 default:
                     break;

             }
         }
    );

}
