//
// Created by DUSK on 2024/12/17.
//

#include "ui.hpp"
#include "GUI.hpp"
#ifdef GUI_ENABLE
#include "beep.h"
#include "led.h"
#endif
lv_chart_series_t *temp;
lv_chart_series_t *humi;
LV_Timer timer;

auto Screen::init() -> void {
  Component::set_parent(gui->main.screen);

  // 按钮初始化
  Button button;
  button.init_font(&lv_customer_font_SourceHanSerifSC_Regular_15);
  button.init(gui->main.btn_test, gui->main.btn_test_label, 100, 600, 100, 100, "蜂鸣器");

  button.init(gui->main.btn_led, gui->main.btn_led_label, 250, 600, 100, 100, "灯光");

  button.init(gui->main.btn_DHT11_value, gui->main.btn_DHT11_value_label, 0, 100, 80, 80, "温湿度");

  Chart::init(gui->main.chart_DHT11_temp, 50, -300, 360, 100, 64);
  Chart::set_range(0, 100);
  Chart::set_div_count(5,7);
  Chart::set_axis_tick(gui->main.chart_DHT11_temp, LV_CHART_AXIS_PRIMARY_Y, 5, 3, 4, 2, true, 40);


  Chart::init(gui->main.chart_DHT11_humi, 50, -200, 360, 100, 64);
  Chart::set_range(0, 100);
  Chart::set_div_count(5,7);
  Chart::set_axis_tick(gui->main.chart_DHT11_humi, LV_CHART_AXIS_PRIMARY_Y, 5, 3, 4, 2, true, 40);

  temp = lv_chart_add_series(gui->main.chart_DHT11_temp, lv_color_hex(0), LV_CHART_AXIS_PRIMARY_Y);
  humi = lv_chart_add_series(gui->main.chart_DHT11_humi, lv_color_hex(0), LV_CHART_AXIS_PRIMARY_Y);
//  Chart::set_next_value(temp, 0, gui->main.chart_DHT11_1);

//  Chart::init(gui->main.chart_DHT11,0,-190,460,100,128);

}

auto Events::init() -> void {
// lambda匿名

  timer.create(timer_fun(
                   Chart::set_next_value(temp, lv_tick_get() & 0x3F, gui->main.chart_DHT11_temp);
                   Chart::set_next_value(humi, lv_tick_get() & 0x3F, gui->main.chart_DHT11_humi);
      ),100);



  bond(gui->main.btn_test, btn_fun(
      []() {
        static bool flag = false;
        flag = !flag;
        if (flag) {
#ifdef GUI_ENABLE
          beep_start();
#endif
        } else {
#ifdef GUI_ENABLE
          beep_stop();
#endif
        }
      }
  ));

  bond(gui->main.btn_led, [](event e) {
         static volatile bool flag2 = false;
         switch (lv_event_get_code(e)) {
           case LV_EVENT_CLICKED:flag2 = !flag2;
             if (flag2) {
#ifdef GUI_ENABLE
               led_start();
#endif
             } else {
#ifdef GUI_ENABLE
               led_stop();
#endif
             }

             break;
           default:break;

         }
       }
  );

  bond(gui->main.btn_DHT11_value, [](event e) {
         static volatile bool flag2 = false;
         switch (lv_event_get_code(e)) {
           case LV_EVENT_CLICKED:flag2 = !flag2;

             if (flag2) {
               timer.resume();
             } else {
               timer.pause();
             }
             break;
           default:break;

         }
       }
  );

}