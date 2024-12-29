//
// Created by DUSK on 2024/12/17.
//

#include "ui.hpp"
#include "GUI.hpp"

#ifdef GUI_ENABLE

#include "beep.h"
#include "led.h"
#include "DCMotor.h"

#endif
lv_chart_series_t *chart_series_temp;
lv_chart_series_t *chart_series_humi;
LV_Timer timer;
lv_chart_cursor_t *cursor;
lv_point_t point = {0, 100};

// 外部声明
extern void start_DHT11();

extern void stop_DHT11();


//#87CEEB
//#FF7F50

auto Screen::init() -> void
{
    Component::set_parent(gui->main.screen);

    // 按钮初始化
    Button button;
    button.init_font(&lv_customer_font_SourceHanSerifSC_Regular_15);

    ImageButton::init(gui->main.imgbtn_led, 150, 330, 80, 80, &_led_off_c_alpha_80x80, &_led_on_c_alpha_80x80);
    ImageButton::init(gui->main.imgbtn_DHT11, 50, 340, 60, 60, &_temp_humi2_alpha_60x60,
                      &_temp_humi_other2_alpha_60x60);


    button.init(gui->main.btn_beep, gui->main.btn_test_label, 100, 620, 100, 100, "蜂鸣器");
    button.init(gui->main.btn_DCMotor, gui->main.btn_DCMotor_label, 100, 470, 100, 100, "拉窗帘/开空调");
    button.init(gui->main.btn_DHT11_settemp, gui->main.btn_DHT11_settemp_label, 250, 470, 100, 100, "设定温度阈值");

    Chart::init(gui->main.chart_DHT11_temp_humi, 10, -240, 400, 280, 64);
    Chart::set_range(0, 100);
    Chart::set_div_count(5, 7);
    Chart::set_axis_tick(gui->main.chart_DHT11_temp_humi, LV_CHART_AXIS_PRIMARY_Y, 5, 3, 4, 2, true, 40);
    // 添加数据

    Chart::add_series(chart_series_temp, lv_color_hex(0x30C0D0), LV_CHART_AXIS_PRIMARY_Y);
    Chart::add_series(chart_series_humi, lv_color_hex(0xC678DD), LV_CHART_AXIS_PRIMARY_Y);
    Chart::add_cursor(gui->main.chart_DHT11_temp_humi, lv_palette_main(LV_PALETTE_GREEN), LV_DIR_RIGHT, cursor);
    Chart::set_cursor_pos(gui->main.chart_DHT11_temp_humi, cursor, &point);
    Chart::set_style_bg_color(gui->main.chart_DHT11_temp_humi, lv_palette_main(LV_PALETTE_ORANGE), LV_PART_SCROLLBAR);




//  Chart::set_next_value(temp, 0, gui->main.chart_DHT11_1);

//  Chart::init(gui->main.chart_DHT11,0,-190,460,100,128);

}

auto Events::init() -> void
{
// lambda匿名

    timer.create(timer_fun(
                         UI::add_temp_data(lv_tick_get() & 0x3F);
                         UI::add_humi_data(lv_tick_get() / 3 & 0x3F);
                 ), 100);


    bond(gui->main.btn_beep, btn_fun(
            []()
            {
                static bool flag = false;
                flag = !flag;
                if (flag)
                {
#ifdef GUI_ENABLE
                    beep_start();
#endif
                }
                else
                {
#ifdef GUI_ENABLE
                    beep_stop();
#endif
                }
            }
    ));


    bond(gui->main.imgbtn_led, imgbtn_fun2(
                 []()
                 {
#ifdef GUI_ENABLE
                     led_start();
#endif
                 }, []()
                 {
#ifdef GUI_ENABLE
                     led_stop();
#endif
                 }
         )
    );


    bond(gui->main.btn_DCMotor, [](event e)
         {
             static volatile bool flag2 = false;
             switch (lv_event_get_code(e))
             {
                 case LV_EVENT_CLICKED:
                     flag2 = !flag2;

                     if (flag2)
                     {
#ifdef GUI_ENABLE
                         DCMotor_forward(1000);
#endif
                     }
                     else
                     {
#ifdef GUI_ENABLE
//                         DCMotor_reverse(300);
                         DCMotor_stop();
#endif
                     }
                     break;
                 default:
                     break;

             }
         }
    );


}


// 添加温度数据
auto UI::add_temp_data(float temp) -> void
{
    Chart::set_next_value(chart_series_temp, (Coord) temp, GUI_Base::get_ui()->main.chart_DHT11_temp_humi);
}

// 添加湿度数据
auto UI::add_humi_data(float humi) -> void
{
    Chart::set_next_value(chart_series_humi, (Coord) humi, GUI_Base::get_ui()->main.chart_DHT11_temp_humi);
}
