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
LV_Timer temp_threshold_timer;
lv_chart_cursor_t *cursor;
lv_point_t cursor_point = {0, 100};
uint16_t temp_threshold;// 温度阈值

// 外部声明
extern void start_DHT11();

extern void stop_DHT11();
extern void start_ACC();

extern void stop_ACC();

extern void switch_sensor(bool type);

extern const float &get_temp();

//#87CEEB
//#FF7F50
/***************************一些函数********************************/
void create_roller(Coord x, Coord y)
{
    Roller::init(GUI_Base::get_ui()->main.roller, x, y, 40, 100, "0\n1\n2\n3\n4\n5\n6\n7\n8\n9");
    Roller::set_selected_text_color(lv_palette_main(LV_PALETTE_ORANGE));
    Roller::set_selected_text_font(&lv_customer_font_SourceHanSerifSC_Regular_18);

    Roller::init(GUI_Base::get_ui()->main.roller2, x + 50, y, 40, 100, "0\n1\n2\n3\n4\n5\n6\n7\n8\n9");
    Roller::set_selected_text_color(lv_palette_main(LV_PALETTE_ORANGE));
    Roller::set_selected_text_font(&lv_customer_font_SourceHanSerifSC_Regular_18);
}

void destroy_roller()
{
    Roller::destroy(GUI_Base::get_ui()->main.roller);
    Roller::destroy(GUI_Base::get_ui()->main.roller2);
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



    // 480-270=210  80,130
    button.init(gui->main.btn_DCMotor, gui->main.btn_DCMotor_label, 65, 470, 90, 90, "步进电机");
    button.init(gui->main.btn_beep, gui->main.btn_beep_label, 195, 470, 90, 90, "蜂鸣器");
    button.init(gui->main.btn_DHT11_set_temp_thresold, gui->main.btn_DHT11_settemp_label, 325, 470, 90, 90,
                "设定温度\n\t  阈值");

    button.init(gui->main.btn_switch_DHT_acc, gui->main.btn_switch_DHT_acc_label, 65, 580, 90, 90, "切换传感器");
    button.init(gui->main.btn_music, gui->main.btn_music_label, 195, 580, 90, 90, "音乐播放");
    button.init(gui->main.btn_ACC, gui->main.btn_ACC_label, 325, 580, 90, 90, "启用加速度\n\t  传感器");

    button.init(gui->main.btn_enable_threshold, gui->main.btn_enable_threshold_label, 65, 690, 90, 90,
                "启用温度\n\t  阈值");
    button.init(gui->main.btn_drag, gui->main.btn_drag_label, 195, 690, 90, 90, "拖拽");
    button.init(gui->main.btn_ensure, gui->main.btn_ensure_label, 325, 690, 90, 90, "确定");

    Chart::init(gui->main.chart_DHT11_temp_humi, 10, -240, 400, 280, 64);
    Chart::set_range(0, 100);
    Chart::set_div_count(5, 7);
    Chart::set_axis_tick(gui->main.chart_DHT11_temp_humi, LV_CHART_AXIS_PRIMARY_Y, 5, 3, 4, 2, true, 40);
    // 添加数据

    Chart::add_series(chart_series_temp, lv_color_hex(0x30C0D0), LV_CHART_AXIS_PRIMARY_Y);
    Chart::add_series(chart_series_humi, lv_color_hex(0xC678DD), LV_CHART_AXIS_PRIMARY_Y);

    Chart::set_style_bg_color(gui->main.chart_DHT11_temp_humi, lv_palette_main(LV_PALETTE_ORANGE), LV_PART_SCROLLBAR);


}

auto Events::init() -> void
{
// lambda匿名

    temp_threshold_timer.create(timer_fun(
#ifdef GUI_ENABLE
                                        // 如果温度大于给定的阈值，蜂鸣器响
                                        if (get_temp()>temp_threshold)
                                {
                                        beep_start();
                                }
#endif
                                ), 2000);


    bond(gui->main.btn_beep, btn_fun(
            []()
            {
                static bool flag = false;
                flag = !flag;
                if (flag)
                {
                    Text::set_text_color(lv_palette_main(LV_PALETTE_RED), gui->main.btn_beep_label);
#ifdef GUI_ENABLE
                    beep_start();
#endif
                }
                else
                {
                    Text::set_text_color(lv_color_black(), gui->main.btn_beep_label);
#ifdef GUI_ENABLE
                    beep_stop();
#endif
                }
            }
    ));


    bond(gui->main.imgbtn_led, imgbtn_fun2(
                 []()
                 {
                     Roller::enable_drag(gui->main.roller);
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
                 []()
                 {
#ifdef GUI_ENABLE
                     start_DHT11();
#endif
                 },
                 []()
                 {
#ifdef GUI_ENABLE
                     stop_DHT11();
#endif
                 }
         )
    );

    // 直流电机
    bond(gui->main.btn_DCMotor, [](event e)
         {
             static volatile bool flag2 = false;
             switch (lv_event_get_code(e))
             {
                 case LV_EVENT_CLICKED:
                     flag2 = !flag2;

                     if (flag2)
                     {
                         Text::set_text_color(lv_palette_main(LV_PALETTE_RED), gui->main.btn_DCMotor_label);
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
                         Text::set_text_color(lv_color_black(), gui->main.btn_DCMotor_label);
                     }
                     break;
                 default:
                     break;

             }
         }
    );

    // 切换加速度传感器、温湿度传感器显示
    bond(gui->main.btn_switch_DHT_acc, [](event e)
    {
        static volatile bool flag3 = false;
        switch (lv_event_get_code(e))
        {
            case LV_EVENT_CLICKED:
                flag3 = !flag3;

                if (flag3)
                {
                    Text::set_text_color(lv_palette_main(LV_PALETTE_RED), gui->main.btn_switch_DHT_acc_label);
#ifdef GUI_ENABLE
                    //ACC
                    switch_sensor(true);
                    Chart::set_range(-360, 360, gui->main.chart_DHT11_temp_humi);


#endif
                }
                else
                {
                    Text::set_text_color(lv_color_black(), gui->main.btn_switch_DHT_acc_label);
#ifdef GUI_ENABLE
                    switch_sensor(false);
                    Chart::set_range(0, 100, gui->main.chart_DHT11_temp_humi);
#endif
                }
                for (int i = 0; i < 64; i++)
                {
                    Chart::set_next_value(chart_series_temp, 0, gui->main.chart_DHT11_temp_humi);
                    Chart::set_next_value(chart_series_humi, 0, gui->main.chart_DHT11_temp_humi);
                }
                break;
            default:
                break;

        }
    });

    // 启用温湿度传感器
    bond(gui->main.btn_music, [](event e)
    {
        static volatile bool flag = false;
        switch (lv_event_get_code(e))
        {
            case LV_EVENT_CLICKED:
                flag = !flag;

                if (flag)
                {
#ifdef GUI_ENABLE
                    start_DHT11();
#endif
                    Text::set_text_color(lv_palette_main(LV_PALETTE_RED), gui->main.btn_music_label);
                }
                else
                {
#ifdef GUI_ENABLE
                    stop_DHT11();
#endif

                    Text::set_text_color(lv_color_black(), gui->main.btn_music_label);
                }

                break;
            default:
                break;

        }
    });

    // 启用加速度传感器
    bond(gui->main.btn_ACC, [](event e)
    {
        static volatile bool flag = false;
        switch (lv_event_get_code(e))
        {
            case LV_EVENT_CLICKED:
                flag = !flag;

                if (flag)
                {
#ifdef GUI_ENABLE
                    start_ACC();
#endif
                    Text::set_text_color(lv_palette_main(LV_PALETTE_RED), gui->main.btn_ACC_label);
                }
                else
                {
#ifdef GUI_ENABLE
                    stop_ACC();
#endif
                    Text::set_text_color(lv_color_black(), gui->main.btn_ACC_label);
                }

                break;
            default:
                break;

        }
    });

    // 设定温度阈值
    bond(gui->main.btn_DHT11_set_temp_thresold, btn_fun
    (
            []()
            {
                // 创建滚轮
                if (!gui->main.roller) { create_roller(320, 330); }
            }
    ));

    // 启用温度阈值
    bond(gui->main.btn_enable_threshold, [](event e)
    {
        static volatile bool flag = false;
        switch (lv_event_get_code(e))
        {
            case LV_EVENT_CLICKED:
                flag = !flag;

                if (flag)
                {
                    temp_threshold_timer.resume();
                    Text::set_text_color(lv_palette_main(LV_PALETTE_RED), gui->main.btn_enable_threshold_label);
                }
                else
                {
                    temp_threshold_timer.pause();
                    Text::set_text_color(lv_color_black(), gui->main.btn_enable_threshold_label);
                }

                break;
            default:
                break;

        }
    });

    // 拖拽按钮
    bond(gui->main.btn_drag, [](event e)
    {
        static volatile bool flag4 = false;
        switch (lv_event_get_code(e))
        {
            case LV_EVENT_CLICKED:

                if (gui->main.roller)
                {
                    flag4 = !flag4;
                    if (flag4)
                    {

                        Roller::enable_drag(gui->main.roller);
                        Roller::enable_drag(gui->main.roller2);

                        Text::set_text_color(lv_palette_main(LV_PALETTE_RED), gui->main.btn_drag_label);
                    }
                    else
                    {
                        Roller::disable_drag(gui->main.roller);
                        Roller::disable_drag(gui->main.roller2);
                        Text::set_text_color(lv_color_black(), gui->main.btn_drag_label);
                    }
                }

                break;
            default:
                break;

        }
    });


    // 确定按钮
    bond(gui->main.btn_ensure, btn_fun(
            []()
            {
                // 确保不会出现空指针引用
                if (gui->main.roller && gui->main.roller2)
                {
                    uint16_t temp = Roller::get_selected_option(gui->main.roller);
                    uint16_t temp2 = Roller::get_selected_option(gui->main.roller2);
                    temp_threshold = temp * 10 + temp2;
                    cursor_point.y = (99 - temp_threshold) / 100.0f * 280;

                    if (cursor)
                    {
                        // 未知原因，直接使用Chart::set_cursor_pos会卡死
                        cursor->pos.y = cursor_point.y;
                        cursor->pos_set = 1;
                        lv_chart_refresh(gui->main.chart_DHT11_temp_humi);
                    }
                    else
                    {
                        Chart::add_cursor(gui->main.chart_DHT11_temp_humi, lv_palette_main(LV_PALETTE_RED),
                                          LV_DIR_RIGHT, cursor);
                        Chart::set_cursor_pos(gui->main.chart_DHT11_temp_humi, cursor, &cursor_point);
                    }
                    destroy_roller();
                }

            }
    ));


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


