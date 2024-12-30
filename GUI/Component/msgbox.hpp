//
// Created by fairy on 2024/12/30 14:56.
//
#ifndef SIMULATOR_MSGBOX_HPP
#define SIMULATOR_MSGBOX_HPP

#include "component.hpp"

class MsgBox : public Component
{
public:
    static inline void init(Obj msgbox, const char *title= nullptr, const char *text= nullptr, const char **button_text= nullptr,bool add_close_button= true);
};

void MsgBox::init(Obj msgbox, const char *title, const char *text, const char **button_text,bool add_close_button)
{
    msgbox = lv_msgbox_create(_parent, title, text, button_text, add_close_button);
    _obj = msgbox;
}



#endif //SIMULATOR_MSGBOX_HPP