//
// Created by fairy on 2024/12/30 02:04.
//
#ifndef SIMULATOR_ROLLER_HPP
#define SIMULATOR_ROLLER_HPP

#include "component.hpp"

class Roller : public Component
{
public:
    static inline void init(Obj roller,Obj parent=_parent);
    static inline void init(Obj roller, Coord x, Coord y, Coord w, Coord h,const char*options= nullptr,lv_roller_mode_t mode = LV_ROLLER_MODE_INFINITE );
    static inline auto set_options(const char*options,lv_roller_mode_t mode = LV_ROLLER_MODE_INFINITE,Obj roller=_obj)->void ;
    static inline auto get_selected_option(Obj roller=_obj)->uint16_t ;
    static inline void set_selected_text_color(lv_color_t color,Obj roller=_obj);
    static inline void set_selected_text_font(lv_font_t* font,Obj roller=_obj);
};

void Roller::init(Obj roller,Obj parent)
{
    roller = lv_roller_create(parent);
    _obj = roller;
}
void Roller::init(Obj roller, Coord x, Coord y, Coord w, Coord h,const char*options,lv_roller_mode_t mode)
{
    roller = lv_roller_create(lv_scr_act());
    lv_obj_set_pos(roller, x, y);
    lv_obj_set_size(roller, w, h);
    if(options)
        lv_roller_set_options(roller, options, mode);
}

void Roller::set_options(const char*options,lv_roller_mode_t mode,Obj roller)
{
    lv_roller_set_options(roller, options, mode);
}

uint16_t Roller::get_selected_option(Obj roller)
{
    return lv_roller_get_selected(roller);
}

void Roller::set_selected_text_color(lv_color_t color,Obj roller)
{
    lv_obj_set_style_text_color(roller, color, LV_PART_SELECTED);
}

void Roller::set_selected_text_font(lv_font_t* font,Obj roller)
{
    lv_obj_set_style_text_font(roller, font, LV_PART_SELECTED);
}

#endif //SIMULATOR_ROLLER_HPP