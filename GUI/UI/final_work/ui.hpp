//
// Created by DUSK on 2024/12/17.
//

#ifndef FURINA_UI_H
#define FURINA_UI_H

#include "lvgl.h"

/*********************组件*******************/
struct lv_ui_t
{
    using Obj = lv_obj_t *;
    // 主屏幕
    struct
    {
        Obj screen;// 屏幕自身

        Obj btn_beep;// 测试按钮
        Obj btn_beep_label;// 测试按钮

        Obj imgbtn_led;
        Obj imgbtn_volume;


        Obj btn_DCMotor;// 直流电机按钮
        Obj btn_DCMotor_label;

        Obj btn_music;// 温湿度传感器
        Obj btn_music_label;

        Obj btn_ACC;// 加速度传感器
        Obj btn_ACC_label;

        Obj btn_DHT11_set_temp_thresold;// 温湿度阈值设定按钮
        Obj btn_DHT11_set_temp_thresold_label;

        Obj btn_switch_DHT_acc;// 切换温湿度传感器和加速度传感器的按钮
        Obj btn_switch_DHT_acc_label;

        Obj btn_enable_threshold;// 开启温度阈值检测按钮
        Obj btn_enable_threshold_label;

        Obj btn_drag;// 拖拽按钮
        Obj btn_drag_label;

        Obj btn_ensure;// 确定按钮
        Obj btn_ensure_label;

        Obj imgbtn_DHT11;
        Obj btn_DHT11_value_label;//取温湿度的值
        Obj chart_DHT11_temp_humi;
        Obj chart_DHT11_humi;
        Obj chart_DHT11_temp_label;
        Obj chart_DHT11_humi_label;

        Obj msgbox_configure;//弹窗

        Obj slider_motor;// 电机控制条

        // 滚动条
        Obj roller;
        Obj roller2;
        Obj roller_volume;// 音量

        Obj init;

    } main;

};
// 取别名
using lv_ui_t = struct lv_ui_t;

//资源加载
LV_FONT_DECLARE(lv_customer_font_SourceHanSerifSC_Regular_15)
LV_FONT_DECLARE(lv_customer_font_SourceHanSerifSC_Regular_18)
LV_IMG_DECLARE(_led_off_c_alpha_80x80)
LV_IMG_DECLARE(_led_on_c_alpha_80x80)
LV_IMG_DECLARE(_temp_humi2_alpha_60x60)
LV_IMG_DECLARE(_temp_humi_other2_alpha_60x60)
LV_IMG_DECLARE(_volume_c_alpha_60x60)
LV_IMG_DECLARE(_volume_c2_alpha_60x60)

/**
 * @brief 界面接口
 */
class UI
{
public:
    static auto add_temp_data(short temp) -> void;// 添加温度数据
    static auto add_humi_data(short humi) -> void;// 添加湿度数据
private:
};


#endif //FURINA_UI_H
