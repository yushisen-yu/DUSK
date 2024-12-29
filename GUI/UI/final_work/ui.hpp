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

    Obj btn_beep;// 测试按钮
    Obj btn_test_label;// 测试按钮

    Obj imgbtn_led;
    Obj btn_led_label;// 灯光按钮

    Obj btn_DCMotor;
    Obj btn_DCMotor_label;

    Obj btn_DHT11_settemp;
    Obj btn_DHT11_settemp_label;

    Obj imgbtn_DHT11_value;
    Obj btn_DHT11_value_label;//取温湿度的值
    Obj chart_DHT11_temp_humi;
    Obj chart_DHT11_humi;
    Obj chart_DHT11_temp_label;
    Obj chart_DHT11_humi_label;



  } main;

};
// 取别名
using lv_ui_t = struct lv_ui_t;

//资源加载
LV_FONT_DECLARE(lv_customer_font_SourceHanSerifSC_Regular_15)
LV_IMG_DECLARE(_led_off_c_alpha_80x80)
LV_IMG_DECLARE(_led_on_c_alpha_80x80)
LV_IMG_DECLARE(_temp_humi2_alpha_60x60)
LV_IMG_DECLARE(_temp_humi_other2_alpha_60x60)

/**
 * @brief 界面接口
 */
class UI {
public:
    static auto add_temp_data(auto temp)-> void;// 添加温度数据
    static auto add_humi_data(auto humi)->  void;// 添加湿度数据
private:
};


#endif //FURINA_UI_H
