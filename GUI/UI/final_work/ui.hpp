//
// Created by DUSK on 2024/12/17.
//

#ifndef FURINA_UI_H
#define FURINA_UI_H
#include "lvgl.h"


/*********************组件*******************/
struct lv_ui_t {
    using Obj = lv_obj_t *;
    // 主屏幕
    struct {
        Obj screen;// 屏幕自身
    } main;
};
// 取别名
using lv_ui_t = struct lv_ui_t;


#endif //FURINA_UI_H
