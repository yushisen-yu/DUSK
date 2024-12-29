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
extern void start();

extern void stop();

extern void switch_sensor(bool type);

//#87CEEB
//#FF7F50

static void drag_event_handler(lv_event_t* e)
{
    lv_obj_t* obj = lv_event_get_target(e); //获取事件产生的对象
    lv_indev_t* indev = lv_indev_get_act();  //获取活动界面输入设备
    lv_point_t vect;
    lv_indev_get_vect(indev, &vect); //获取vect point
    lv_coord_t x = lv_obj_get_x(obj) + vect.x; //计算x
    lv_coord_t y = lv_obj_get_y(obj) + vect.y; // 计算y
    lv_obj_set_pos(obj, x, y); //移动对象到x,y
}

auto Screen::init() -> void
{
    Component::set_parent(gui->main.screen);

    // 按钮初始化
    Button button;
    button.init_font(&lv_customer_font_SourceHanSerifSC_Regular_15);

    ImageButton::init(gui->main.imgbtn_led, 150, 330, 80, 80, &_led_off_c_alpha_80x80, &_led_on_c_alpha_80x80);
    ImageButton::init(gui->main.imgbtn_DHT11, 50, 340, 60, 60, &_temp_humi2_alpha_60x60,
                      &_temp_humi_other2_alpha_60x60);


    button.init(gui->main.btn_beep, gui->main.btn_test_label, 100, 620, 90, 90, "蜂鸣器");
    button.init(gui->main.btn_DCMotor, gui->main.btn_DCMotor_label, 100, 470, 90, 90, "拉窗帘/开空调");
    button.init(gui->main.btn_DHT11_settemp, gui->main.btn_DHT11_settemp_label, 250, 470, 90, 90, "设定温度阈值");
    button.init(gui->main.btn_switch_DHT_acc, gui->main.btn_switch_DHT_acc_label, 250, 620, 90, 90, "切换传感器");
    button.init(gui->main.btn_ensure, gui->main.btn_ensure_label, 350, 620, 90, 90, "确定");

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

    // 滚动条使用
    Roller::init(gui->main.roller,300, 500,40, 100, "0\n1\n2\n3\n4\n5\n6\n7\n8\n9");
    Roller::set_selected_text_color(lv_palette_main(LV_PALETTE_ORANGE));
    Roller::set_selected_text_font( &lv_customer_font_SourceHanSerifSC_Regular_18);

    lv_obj_add_event_cb(gui->main.roller, drag_event_handler,LV_EVENT_PRESSING, nullptr);

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

    bond(gui->main.imgbtn_DHT11, imgbtn_fun2(
            [](){
#ifdef GUI_ENABLE
                start();
#endif
                },
            [](){
#ifdef GUI_ENABLE
                stop();
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

    bond(gui->main.btn_switch_DHT_acc,[](event e)
    {
        static volatile bool flag3 = false;
        switch (lv_event_get_code(e))
        {
            case LV_EVENT_CLICKED:
                flag3 = !flag3;

                if (flag3)
                {
#ifdef GUI_ENABLE
                    //ACC
                    switch_sensor(true);
                    Chart::set_range(-360, 360,gui->main.chart_DHT11_temp_humi);


#endif
                }
                else
                {
#ifdef GUI_ENABLE
                    switch_sensor(false);
                    Chart::set_range(0, 100,gui->main.chart_DHT11_temp_humi);
#endif
                }
                for(int i=0;i<64;i++)
                {
                    Chart::set_next_value(chart_series_temp, 0, gui->main.chart_DHT11_temp_humi);
                    Chart::set_next_value(chart_series_humi, 0, gui->main.chart_DHT11_temp_humi);
                }
                break;
            default:
                break;

        }
    });


}


// 添加温度数据
auto UI::add_temp_data(short temp) -> void
{

    Chart::set_next_value(chart_series_temp, temp, GUI_Base::get_ui()->main.chart_DHT11_temp_humi);
}

// 添加湿度数据
auto UI::add_humi_data(short humi) -> void
{
    Chart::set_next_value(chart_series_humi, humi, GUI_Base::get_ui()->main.chart_DHT11_temp_humi);
}


