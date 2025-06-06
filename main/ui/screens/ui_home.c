// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.2
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "../ui.h"
#include "app_wifi.h"
#include <stdio.h>

void ui_home_screen_init(void)
{
    ui_home = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_home, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Panel1 = lv_obj_create(ui_home);
    lv_obj_set_width(ui_Panel1, 240);
    lv_obj_set_height(ui_Panel1, 240);
    lv_obj_set_align(ui_Panel1, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel1, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    // lv_obj_set_style_bg_color(ui_Panel1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Panel1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Panel1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Panel1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Panel1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Panel1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_hour = lv_label_create(ui_Panel1);
    lv_obj_set_width(ui_hour, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_hour, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_hour, -60);
    lv_obj_set_y(ui_hour, 0);
    lv_obj_set_align(ui_hour, LV_ALIGN_BOTTOM_MID);
    lv_label_set_text(ui_hour, "23");
    lv_obj_set_style_text_color(ui_hour, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_hour, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_hour, &ui_font_ComfortaaBold75, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_min = lv_label_create(ui_Panel1);
    lv_obj_set_width(ui_min, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_min, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_min, 60);
    lv_obj_set_y(ui_min, 0);
    lv_obj_set_align(ui_min, LV_ALIGN_BOTTOM_MID);
    lv_label_set_text(ui_min, "59");
    lv_obj_set_style_text_color(ui_min, lv_color_hex(0xF1BA3B), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_min, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_min, &ui_font_ComfortaaBold75, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_date = lv_label_create(ui_Panel1);
    lv_obj_set_width(ui_date, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_date, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_date, 25);
    lv_obj_set_y(ui_date, 15);
    lv_obj_set_align(ui_date, LV_ALIGN_CENTER);
    lv_label_set_text(ui_date, "02/20");
    lv_obj_set_style_text_color(ui_date, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_date, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_date, &ui_font_OPPOSansH18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_weekday = lv_label_create(ui_Panel1);
    lv_obj_set_width(ui_weekday, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_weekday, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_weekday, 85);
    lv_obj_set_y(ui_weekday, 15);
    lv_obj_set_align(ui_weekday, LV_ALIGN_CENTER);
    lv_label_set_text(ui_weekday, "周日");
    lv_obj_set_style_text_color(ui_weekday, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_weekday, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_weekday, &ui_font_OPPOSansH25, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_temp = lv_label_create(ui_Panel1);
    lv_obj_set_width(ui_temp, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_temp, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_temp, 60);
    lv_obj_set_y(ui_temp, 30);
    lv_obj_set_align(ui_temp, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_temp, "15°");
    lv_obj_set_style_text_color(ui_temp, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_temp, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_temp, &ui_font_OPPOSansL70, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_weather = lv_label_create(ui_Panel1);
    lv_obj_set_width(ui_weather, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_weather, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_weather, -60);
    lv_obj_set_y(ui_weather, 15);
    lv_obj_set_align(ui_weather, LV_ALIGN_CENTER);
    lv_label_set_text(ui_weather, "多云");
    lv_obj_set_style_text_color(ui_weather, lv_color_hex(0xF1BA3B), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_weather, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_weather, &ui_font_OPPOSansH25, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_colon = lv_label_create(ui_Panel1);
    lv_obj_set_width(ui_colon, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_colon, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_colon, 0);
    lv_obj_set_y(ui_colon, -8);
    lv_obj_set_align(ui_colon, LV_ALIGN_BOTTOM_MID);
    lv_label_set_text(ui_colon, ":");
    lv_obj_set_style_text_color(ui_colon, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_colon, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_colon, &ui_font_ComfortaaBold75, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_weathershow = lv_img_create(ui_Panel1);
    lv_obj_set_width(ui_weathershow, 110);
    lv_obj_set_height(ui_weathershow, 110);
    lv_obj_set_x(ui_weathershow, -60);
    lv_obj_set_y(ui_weathershow, -54);
    lv_obj_set_align(ui_weathershow, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_weathershow, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_weathershow, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_img_set_zoom(ui_weathershow, 255);

    lv_obj_add_event_cb(ui_home, ui_event_home, LV_EVENT_ALL, NULL);

}
