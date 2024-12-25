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
    Obj btn_test;// 测试按钮
    Obj btn_test_label;// 测试按钮
    Obj btn_led;
    Obj btn_led_label;// 灯光按钮
    Obj btn_DHT11_value;
    Obj btn_DHT11_value_label;//去温湿度的值
    Obj chart_DHT11_temp;
    Obj chart_DHT11_humi;

  } main;

};
// 取别名
using lv_ui_t = struct lv_ui_t;

//资源加载
LV_FONT_DECLARE(lv_customer_font_SourceHanSerifSC_Regular_15)

#endif //FURINA_UI_H
