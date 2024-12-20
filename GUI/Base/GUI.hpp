//
// Created by fairy on 2024/10/17 18:53.
//
#ifndef FURINA_GUI_HPP
#define FURINA_GUI_HPP

#include <cstdio>

/* 预编译命令 */
#if 1
// 头文件
#include "LV_Timer.hpp"
#include "events.hpp"
#include "chart.hpp"
#include "slider.hpp"
#include "image.hpp"
#include "text.hpp"
#include "button.hpp"
#include "imageButton.hpp"
#include "FPS.hpp"
// 宏定义
#define SIMPLE_FPS 1 // 启用简单FPS，减少一点点FPS显示对实际帧率的影响。不严谨地对于某个简单的测试来说，从30ms减少到了21ms

/*匿名命名空间，专治各种函数变量暴露狂*/
namespace
{
    constexpr uint16_t MY_DISP_HOR_RES = 480;
    constexpr uint16_t MY_DISP_VER_RES = 800;
    constexpr uint16_t MY_DISP_BUF_SIZE = 30;

    lv_disp_drv_t disp_drv{};
    lv_indev_drv_t indev_drv{};
    lv_indev_t *indev_touchpad;
}

static inline auto LVGL_LCD_FSMC_DMA_pCallback() -> void
{
    disp_drv.draw_buf->flushing = 0;
    disp_drv.draw_buf->flushing_last = 0;
}


/**
 * @brief GUI类
 */
class GUI : public GUI_Base
{
public:
    template<void (*disp_flush)(uint16_t, uint16_t, uint16_t, uint16_t, const uint16_t *),
            uint8_t (*touchpad_read)(int16_t *x, int16_t *y) = nullptr>
    static auto init() -> void;

    static inline auto handler() -> void;

    // 刷新回调
    static inline auto LVGL_LCD_FSMC_DMA_pCallback() -> void;

#ifdef  ARM_MATH_CM4
private:
#else
    public:
#endif

    static auto resource_init() -> void;// 初始化界面

    template<void (*disp_flush)(uint16_t, uint16_t, uint16_t, uint16_t, const uint16_t *),
            uint8_t (*touchpad_read)(int16_t *x, int16_t *y) = nullptr>
    static auto disp_drv_init() -> void;

private:
    static inline lv_disp_drv_t disp_drv{};
};


/**
 * @brief GUI事件循环
 */
auto GUI::handler() -> void
{
    lv_task_handler();
}


/**
 * @brief 回调函数
 * @note 用于DMA等刷新完成后通知LVGL
 */
auto GUI::LVGL_LCD_FSMC_DMA_pCallback() -> void
{
    disp_drv.draw_buf->flushing = 0;
    disp_drv.draw_buf->flushing_last = 0;
}


/**
 * @brief 初始化显示驱动
 * @tparam flush 涂色函数，有LCD驱动提供
 * @note 为了让lambda表达式可以不用捕获外部函数，只能使用函数模板。如果使用函数指针来传递就必须要显示捕获
 */
template<void (*disp_flush)(uint16_t, uint16_t, uint16_t, uint16_t, const uint16_t *),
        uint8_t (*touchpad_read)(int16_t *x, int16_t *y)>
auto GUI::disp_drv_init() -> void
{
    // 在缓冲数组总大小同等的情况下，双缓冲明显优于单缓冲
    static lv_disp_draw_buf_t draw_buf_dsc;
    static lv_color_t buf_2_1[MY_DISP_HOR_RES * MY_DISP_BUF_SIZE];
    static lv_color_t buf_2_2[MY_DISP_HOR_RES * MY_DISP_BUF_SIZE];
    lv_disp_draw_buf_init(&draw_buf_dsc, buf_2_1, buf_2_2,
                          MY_DISP_HOR_RES * MY_DISP_BUF_SIZE);   /*Initialize the display buffer*/

    lv_disp_drv_init(&disp_drv);                    /*Basic initialization*/

    disp_drv.hor_res = MY_DISP_HOR_RES;
    disp_drv.ver_res = MY_DISP_VER_RES;
    disp_drv.flush_cb = [](lv_disp_drv_t *, const lv_area_t *area, lv_color_t *color_p)
    {
        disp_flush(area->x1, area->y1, area->x2, area->y2, (const uint16_t *) color_p);
        // 仅限不使用DMA的情况
        disp_drv.draw_buf->flushing = 0;
        disp_drv.draw_buf->flushing_last = 0;
    };
    disp_drv.draw_buf = &draw_buf_dsc;

    lv_disp_drv_register(&disp_drv);

    // 触摸屏初始化
    /*Register a touchpad input device*/
    if constexpr (touchpad_read != nullptr)
    {
        lv_indev_drv_init(&indev_drv);
        indev_drv.type = LV_INDEV_TYPE_POINTER;
        indev_drv.read_cb = [](lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
        {
            static lv_coord_t last_x = 0;
            static lv_coord_t last_y = 0;

            /*Save the pressed coordinates and the state*/
            if (touchpad_read(&last_x, &last_y))
            {
                data->state = LV_INDEV_STATE_PR;
            } else
            {
                data->state = LV_INDEV_STATE_REL;
            }

            /*Set the last pressed coordinates*/
            data->point.x = last_x;
            data->point.y = last_y;
        };
        indev_touchpad = lv_indev_drv_register(&indev_drv);
    }
}


template<void (*disp_flush)(uint16_t, uint16_t, uint16_t, uint16_t, const uint16_t *),
        uint8_t (*touchpad_read)(int16_t *x, int16_t *y)>
auto GUI::init() -> void
{
    /*****初始化设备*****/
#ifdef ARM_MATH_CM4
    disp_drv_init<disp_flush, touchpad_read>();
#endif

    /*****初始化GUI组件*****/
    GUI::resource_init();
}


#endif
#endif //FURINA_GUI_HPP