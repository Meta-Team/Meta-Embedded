//
// Created by 钱晨 on 7/19/21.
//

#include "referee_UI_logic.h"
#include "shell.h"
#include "common_macro.h"

char RefereeUILG::cap_name[3] = {'c', 'a', 'p'};
char RefereeUILG::cap_title[30];
unsigned long RefereeUILG::cap_update_time;

char RefereeUILG::top_name[3] = {'t', 'o', 'p'};
char RefereeUILG::top_title[30];
unsigned long RefereeUILG::top_update_time;

char RefereeUILG::dodge_name[3] = {'d', 'g', 'e'};
char RefereeUILG::dodge_title[30];
unsigned long RefereeUILG::dodge_update_time;

char RefereeUILG::gun_indicator_name[3] = {'g', 'u', 'n'};
RefereeUISKD::UI_point RefereeUILG::gun_start_point;
RefereeUISKD::UI_point RefereeUILG::gun_end_point;

char RefereeUILG::chassis_indicator_1_name[3] = {'c', 'h', '1'};
RefereeUISKD::UI_point RefereeUILG::chassis_1_start_point;
RefereeUISKD::UI_point RefereeUILG::chassis_1_end_point;

char RefereeUILG::chassis_indicator_2_name[3] = {'c', 'h', '2'};
RefereeUISKD::UI_point RefereeUILG::chassis_2_start_point;
RefereeUISKD::UI_point RefereeUILG::chassis_2_end_point;

unsigned long RefereeUILG::angle_update_time;

char RefereeUILG::main_enemy_name[3] = {'e', 'n', 'm'};
RefereeUISKD::UI_point RefereeUILG::main_enemy;


void RefereeUILG::init() {
    Referee::remove_all();
    init_data_UI();
    init_chassis_UI();
    init_vision_UI();
}



void RefereeUILG::revise_cap(float cap_volt) {
    if(WITHIN_RECENT_TIME(cap_update_time, 400)) return;
    Shell::snprintf(cap_title, sizeof(cap_title), "CAP  VOLT  : %.1f", cap_volt);
    RefereeUISKD::color_t title_color;
    if(cap_volt >15.0f) {
        title_color = RefereeUISKD::GREEN;
    } else {
        title_color = RefereeUISKD::YELLOW;
    }
    RefereeUISKD::revise_character(cap_name, cap_title, title_color);
    cap_update_time = SYSTIME;
}

void RefereeUILG::toggle_top(bool top_stat) {
    if(WITHIN_RECENT_TIME(top_update_time, 400)) return;
    RefereeUISKD::color_t title_color;
    if(top_stat) {
        Shell::snprintf(top_title, sizeof(top_title), "CASE STAT  : OPEN [R]");
        title_color = RefereeUISKD::GREEN;
    } else {
        Shell::snprintf(top_title, sizeof(top_title), "CASE STAT  : CLOSE[R]");
        title_color = RefereeUISKD::ACCENT_COLOR;
    }
    RefereeUISKD::revise_character(cap_name, cap_title, title_color);
    top_update_time = SYSTIME;
}

void RefereeUILG::toggle_dodge(bool dodge_stat) {
    if(WITHIN_RECENT_TIME(dodge_update_time, 400)) return;
    RefereeUISKD::color_t title_color;
    if(dodge_stat) {
        Shell::snprintf(dodge_title, sizeof(dodge_title),    "DODGE MODE : ON   [X]");
        title_color = RefereeUISKD::GREEN;
    } else {
        Shell::snprintf(dodge_title, sizeof(dodge_title),    "DODGE MODE : OFF  [X]");
        title_color = RefereeUISKD::ACCENT_COLOR;
    }
    RefereeUISKD::revise_character(dodge_name, dodge_title, title_color);
    dodge_update_time = SYSTIME;
}

void RefereeUILG::set_chassis_angle(float angle) {
    if(WITHIN_RECENT_TIME(angle_update_time,25)) return;
    double x_offset = 25.0f*sin(angle);
    double y_offset = 25.0f*cos(angle);
    chassis_1_start_point.x   = (uint32_t) (960.0f - x_offset);
    chassis_1_end_point.x     = (uint32_t) (960.0f + x_offset);
    chassis_1_start_point.y   = (uint32_t) (200.0f - y_offset);
    chassis_1_end_point.y     = (uint32_t) (200.0f + y_offset);
//    chassis_2_end_point.x     = (uint32_t) (960.0f + 30.0f*sin(angle));
//    chassis_2_end_point.y     = (uint32_t) (200.0f + 30.0f*cos(angle));
    RefereeUISKD::revise_line(chassis_indicator_1_name, chassis_1_start_point, chassis_1_end_point);
//    RefereeUISKD::revise_line(chassis_indicator_2_name, chassis_1_start_point, chassis_2_end_point);
    angle_update_time = SYSTIME;
}

void RefereeUILG::set_main_enemy(RefereeUISKD::UI_point enemy_loc){
    main_enemy.x = enemy_loc.x;
    main_enemy.y = enemy_loc.y;
    RefereeUISKD::revise_shape_loc(main_enemy_name, main_enemy, RefereeUISKD::ACCENT_COLOR);
}

void RefereeUILG::init_data_UI() {
    RefereeUISKD::remove_layer(3);
    Shell::snprintf(cap_title, sizeof(cap_title),        "CAP  VOLT  : INIT", 0.0f);
    RefereeUISKD::add_character(cap_name, {100, 540}, RefereeUISKD::GREEN, 3, 15, 2, cap_title);
    Shell::snprintf(dodge_title, sizeof(dodge_title),    "DODGE MODE : OFF  [X]");
    RefereeUISKD::add_character(dodge_name, {100, 570}, RefereeUISKD::ACCENT_COLOR, 3, 15, 2, dodge_title);
    Shell::snprintf(top_title, sizeof(top_title),        "CASE STAT  : INIT [R]");
    RefereeUISKD::add_character(top_name, {100, 600}, RefereeUISKD::ACCENT_COLOR, 3, 15, 2, top_title);
}

void RefereeUILG::init_chassis_UI() {
    RefereeUISKD::remove_layer(2);
    RefereeUILG::gun_end_point = {960, 200};
    RefereeUILG::gun_start_point = {960, 250};
    RefereeUISKD::add_line(gun_indicator_name, 2, RefereeUISKD::GREEN, gun_start_point, gun_end_point, 2);
    RefereeUILG::chassis_1_end_point = {960, 200 + 25};
    RefereeUILG::chassis_1_start_point = {960, 200 - 25};
    RefereeUISKD::add_line(chassis_indicator_1_name, 2, RefereeUISKD::ACCENT_COLOR, chassis_1_start_point, chassis_1_end_point, 30);
//    RefereeUILG::chassis_2_end_point = {960, 230};
//    RefereeUISKD::add_line(chassis_indicator_2_name, 2, RefereeUISKD::ACCENT_COLOR, chassis_1_start_point, chassis_2_end_point, 20);
}

void RefereeUILG::init_vision_UI() {
    RefereeUISKD::remove_layer(1);
    RefereeUILG::main_enemy = {960, 540};
    RefereeUISKD::add_circle(main_enemy_name, 1, RefereeUISKD::ACCENT_COLOR, main_enemy, 10, 10);
}