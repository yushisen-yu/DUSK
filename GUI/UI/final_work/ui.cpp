//
// Created by DUSK on 2024/12/17.
//

#include "ui.hpp"
#include "GUI.hpp"

#ifdef GUI_ENABLE

#include "beep.h"
#include "led.h"
#include "DCMotor.h"
#include "mp3.h"
#include "stm32f4xx_hal.h"

#endif
lv_chart_series_t *chart_series_temp;
lv_chart_series_t *chart_series_humi;
LV_Timer temp_threshold_timer;
LV_Timer blink_timer;
lv_chart_cursor_t *cursor;
lv_point_t cursor_point = {0, 100};
uint16_t temp_threshold;//
//0表示温度，1表示音乐,2表示音量共有6个档
// 实在是懒得搞标志了，除非还有时间.它俩之间的逻辑有些耦合，先不管了
uint8_t roller_mode = 0;
enum class ConfigFlags : uint8_t
{
    TEMP_THRESHOLD = 1 << 0,// 温度阈值
    MUSIC_INDEX = 1 << 1,// 音乐索引
    VOLUME = 1 << 2,// 音量
};

class Config
{
public:
    static uint8_t get_flags() { return flags; }

public:
    // 辅助函数：设置指定标志
    static void set_flag(ConfigFlags flag)
    {
        flags |= static_cast<uint8_t>(flag);
    }

    // 辅助函数：清除指定标志
    static void clear_flag(ConfigFlags flag)
    {
        flags &= ~static_cast<uint8_t>(flag);
    }

    // 辅助函数：获取指定标志
    static uint8_t get_flag(ConfigFlags flag)
    {
        return flags & static_cast<uint8_t>(flag);
    }

    // 辅助函数：翻转指定标志
    static void toggle_flag(ConfigFlags flag)
    {
        flags ^= static_cast<uint8_t>(flag);
    }

private:
    static inline uint8_t flags = 0;
};

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
void create_roller()
{
    Roller::init(GUI_Base::get_ui()->main.roller, 365, 330, 40, 120, "0\n1\n2\n3\n4\n5\n6\n7\n8\n9");
    Roller::set_selected_text_color(lv_palette_main(LV_PALETTE_ORANGE));
    Roller::set_selected_text_font(&lv_customer_font_SourceHanSerifSC_Regular_18);

    Roller::init(GUI_Base::get_ui()->main.roller2, 415, 330, 40, 120, "0\n1\n2\n3\n4\n5\n6\n7\n8\n9");
    Roller::set_selected_text_color(lv_palette_main(LV_PALETTE_ORANGE));
    Roller::set_selected_text_font(&lv_customer_font_SourceHanSerifSC_Regular_18);
}

void destroy_roller()
{
    Roller::destroy(GUI_Base::get_ui()->main.roller);
    Roller::destroy(GUI_Base::get_ui()->main.roller2);
}

void create_volume_roller()
{
    Roller::init(GUI_Base::get_ui()->main.roller_volume, 305, 330, 40, 120, "0\n1\n2\n3\n4\n5\n6");
    Roller::set_selected_text_color(lv_palette_main(LV_PALETTE_ORANGE));
    Roller::set_selected_text_font(&lv_customer_font_SourceHanSerifSC_Regular_18);
}

void destroy_volume_roller()
{
    Roller::destroy(GUI_Base::get_ui()->main.roller_volume);
}

// 弹窗
void create_msgbox()
{
    MsgBox::init(GUI_Base::get_ui()->main.msgbox_configure, "警告", "请先配置当前选项或者取消当前配置！", nullptr, true);
    MsgBox::enable_drag(GUI_Base::get_ui()->main.msgbox_configure);
}

void destroy_msgbox()
{
    MsgBox::destroy(GUI_Base::get_ui()->main.msgbox_configure);
}

auto Screen::init() -> void
{
    Component::set_parent(gui->main.screen);

    // 按钮初始化
    Button button;
    button.init_font(&lv_customer_font_SourceHanSerifSC_Regular_15);

    ImageButton::init(gui->main.imgbtn_DHT11, 50, 340, 60, 60, &_temp_humi2_alpha_60x60,
                      &_temp_humi_other2_alpha_60x60);
    ImageButton::init(gui->main.imgbtn_led, 120, 330, 80, 80, &_led_off_c_alpha_80x80, &_led_on_c_alpha_80x80);
    ImageButton::init(gui->main.imgbtn_volume, 210, 343, 60, 60, &_volume_c_alpha_60x60, &_volume_c2_alpha_60x60);



    // 480-270=210  80,130
    button.init(gui->main.btn_DCMotor, gui->main.btn_DCMotor_label, 65, 470, 90, 90, "步进电机");
    button.init(gui->main.btn_beep, gui->main.btn_beep_label, 195, 470, 90, 90, "蜂鸣器");
    button.init(gui->main.btn_DHT11_set_temp_thresold, gui->main.btn_DHT11_set_temp_thresold_label, 325, 470, 90, 90,
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
                                }else
                                {
                                        beep_stop();
                                }
#endif
                                ), 2000);


    blink_timer.create(timer_fun(
#ifdef GUI_ENABLE
                               static bool flag = false;
                               flag = !flag;
                               // 如果温度大于给定的阈值，蜂鸣器响
                               if (get_temp()>temp_threshold)
                       {
                               if (flag)
                       { led_start(); }
                               else
                       {
                               led_stop();
                       }
                       }
                               else
                       {
                               led_stop();
                       }

#endif
                       ), 200);


    bond(gui
                 ->main.btn_beep, btn_fun(
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


    bond(gui
                 ->main.imgbtn_led, imgbtn_fun2(
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

    bond(gui
                 ->main.imgbtn_DHT11, imgbtn_fun2(
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

    bond(gui
                 ->main.imgbtn_volume, imgbtn_fun2([]()
                                                   {
                                                       if (!gui->main.roller_volume)
                                                       {
                                                           Config::set_flag(ConfigFlags::VOLUME);
                                                           create_volume_roller();

                                                       }

                                                   }, []()
                                                   {
                                                       if (gui->main.roller_volume)
                                                       {
                                                           Config::clear_flag(ConfigFlags::VOLUME);
                                                           destroy_volume_roller();
                                                       }

                                                   }));

// 直流电机
    bond(gui
                 ->main.btn_DCMotor, [](
                 event e
         )
         {
             static volatile bool flag2 = false;
             switch (
                     lv_event_get_code(e)
                     )
             {
                 case LV_EVENT_CLICKED:
                     flag2 = !flag2;

                     if (flag2)
                     {
                         Text::set_text_color(lv_palette_main(LV_PALETTE_RED), gui

                                 ->main.btn_DCMotor_label);
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

                         Text::set_text_color(lv_color_black(), gui

                                 ->main.btn_DCMotor_label);
                     }
                     break;
                 default:
                     break;

             }
         }
    );

// 切换加速度传感器、温湿度传感器显示
    bond(gui
                 ->main.btn_switch_DHT_acc, [](
            event e
    )
         {
             static volatile bool flag3 = false;
             switch (
                     lv_event_get_code(e)
                     )
             {
                 case LV_EVENT_CLICKED:
                     flag3 = !flag3;

                     if (flag3)
                     {
                         Text::set_text_color(lv_palette_main(LV_PALETTE_RED), gui

                                 ->main.btn_switch_DHT_acc_label);
#ifdef GUI_ENABLE
//ACC
                         switch_sensor(true);
                         Chart::set_range(-360, 360, gui->main.chart_DHT11_temp_humi);


#endif
                     }
                     else
                     {
                         Text::set_text_color(lv_color_black(), gui

                                 ->main.btn_switch_DHT_acc_label);
#ifdef GUI_ENABLE
                         switch_sensor(false);
                         Chart::set_range(0, 100, gui->main.chart_DHT11_temp_humi);
#endif
                     }
                     for (
                             int i = 0;
                             i < 64; i++)
                     {
                         Chart::set_next_value(chart_series_temp,
                                               0, gui->main.chart_DHT11_temp_humi);
                         Chart::set_next_value(chart_series_humi,
                                               0, gui->main.chart_DHT11_temp_humi);
                     }
                     break;
                 default:
                     break;

             }
         });

// 启用音乐播放
    bond(gui
                 ->main.btn_music, [](
            event e
    )
         {
             static volatile bool flag = false;
             switch (
                     lv_event_get_code(e)
                     )
             {
                 case LV_EVENT_CLICKED:
                     flag = !flag;

                     if (flag)
                     {
                         if (!gui->main.roller)
                         {
                             Config::set_flag(ConfigFlags::MUSIC_INDEX);// 音乐播放
                             create_roller();

                             Text::set_text_color(lv_palette_main(LV_PALETTE_BLUE), gui

                                     ->main.btn_music_label);
                         }
                     }
                     else
                     {
                         if (gui->main.roller)
                         {
                             Config::clear_flag(ConfigFlags::MUSIC_INDEX);

                             destroy_roller();

                         }
#ifdef GUI_ENABLE

                         mp3Stop();

#endif

                         Text::set_text_color(lv_color_black(), gui

                                 ->main.btn_music_label);
                     }

                     break;
                 default:
                     break;

             }
         });

// 启用加速度传感器
    bond(gui
                 ->main.btn_ACC, [](
            event e
    )
         {
             static volatile bool flag = false;
             switch (
                     lv_event_get_code(e)
                     )
             {
                 case LV_EVENT_CLICKED:
                     flag = !flag;

                     if (flag)
                     {
#ifdef GUI_ENABLE

                         start_ACC();

#endif

                         Text::set_text_color(lv_palette_main(LV_PALETTE_RED), gui

                                 ->main.btn_ACC_label);
                     }
                     else
                     {
#ifdef GUI_ENABLE

                         stop_ACC();

#endif

                         Text::set_text_color(lv_color_black(), gui

                                 ->main.btn_ACC_label);
                     }

                     break;
                 default:
                     break;

             }
         });

// 设定温度阈值
    bond(gui
                 ->main.btn_DHT11_set_temp_thresold, [](
            event e
    )
         {
             static volatile bool flag = false;
             switch (
                     lv_event_get_code(e)
                     )
             {
                 case LV_EVENT_CLICKED:
                     flag = !flag;

                     if (flag)
                     {
// 创建滚轮
                         if (!gui->main.roller)
                         {
                             Config::set_flag(ConfigFlags::TEMP_THRESHOLD);

                             create_roller();

                         }

                         Text::set_text_color(lv_palette_main(LV_PALETTE_RED), gui

                                 ->main.btn_DHT11_set_temp_thresold_label);
                     }
                     else
                     {
// 创建滚轮
                         if (gui->main.roller)
                         {
                             Config::clear_flag(ConfigFlags::TEMP_THRESHOLD);

                             destroy_roller();

                         }

                         Text::set_text_color(lv_color_black(), gui

                                 ->main.btn_DHT11_set_temp_thresold_label);
                     }

                     break;
                 default:
                     break;

             }
         });

// 启用温度阈值
    bond(gui
                 ->main.btn_enable_threshold, [](
            event e
    )
         {
             static volatile bool flag = false;
             switch (
                     lv_event_get_code(e)
                     )
             {
                 case LV_EVENT_CLICKED:
                     flag = !flag;

                     if (flag)
                     {
                         temp_threshold_timer.

                                 resume();

                         blink_timer.

                                 resume();

                         Text::set_text_color(lv_palette_main(LV_PALETTE_RED), gui

                                 ->main.btn_enable_threshold_label);
                     }
                     else
                     {

                         temp_threshold_timer.

                                 pause();

                         blink_timer.

                                 pause();

#ifdef GUI_ENABLE

                         beep_stop();

#endif

                         Text::set_text_color(lv_color_black(), gui

                                 ->main.btn_enable_threshold_label);
                     }

                     break;
                 default:
                     break;

             }
         });

// 拖拽按钮
    bond(gui
                 ->main.btn_drag, [](
            event e
    )
         {
             static volatile bool flag4 = false;
             switch (
                     lv_event_get_code(e)
                     )
             {
                 case LV_EVENT_CLICKED:

                     if (gui->main.roller)
                     {
                         flag4 = !flag4;
                         if (flag4)
                         {

                             Roller::enable_drag(gui
                                                         ->main.roller);
                             Roller::enable_drag(gui
                                                         ->main.roller2);

                             Text::set_text_color(lv_palette_main(LV_PALETTE_RED), gui

                                     ->main.btn_drag_label);
                         }
                         else
                         {
                             Roller::disable_drag(gui
                                                          ->main.roller);
                             Roller::disable_drag(gui
                                                          ->main.roller2);

                             Text::set_text_color(lv_color_black(), gui

                                     ->main.btn_drag_label);
                         }
                     }

                     break;
                 default:
                     break;

             }
         });


// 确定按钮
    bond(gui
                 ->main.btn_ensure, btn_fun(
                 []()
                 {

                     uint16_t temp = Roller::get_selected_option(gui->main.roller);
                     uint16_t temp2 = Roller::get_selected_option(gui->main.roller2);

                     if (Config::get_flag(ConfigFlags::TEMP_THRESHOLD))
                     {
                         // 确保不会出现空指针引用
                         if (gui->main.roller && gui->main.roller2)
                         {

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
                             Button::click(gui->main.btn_DHT11_set_temp_thresold);
                             Config::clear_flag(ConfigFlags::TEMP_THRESHOLD);
                         }
                     }
                     if (Config::get_flag(ConfigFlags::MUSIC_INDEX))
                     {
                         // 音乐播放
                         if (gui->main.roller && gui->main.roller2)
                         {
#ifdef GUI_ENABLE
                             mp3_play_selected(temp * 10 + temp2);
#endif
                             destroy_roller();
                         }
                     }
                     if (Config::get_flag(ConfigFlags::VOLUME))
                     {


#ifdef GUI_ENABLE
                         setMp3Vol(Roller::get_selected_option(gui->main.roller_volume) * 5);
#endif
                         ImageButton::release(gui->main.imgbtn_volume);
                         Config::clear_flag(ConfigFlags::VOLUME);
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


