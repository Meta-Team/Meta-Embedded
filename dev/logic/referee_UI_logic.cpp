//
// Created by 钱晨 on 7/19/21.
//

#include "referee_UI_logic.h"
#include "shell.h"
#include "common_macro.h"

const char RefereeUILG::cap_name[3] = {'c', 'a', 'p'};
char RefereeUILG::cap_title[31];
unsigned long RefereeUILG::cap_update_time;

const char RefereeUILG::bullet_case_name[3] = {'t', 'o', 'p'};
char RefereeUILG::bullet_case_title[31];
RefereeUISKD::color_t RefereeUILG::bullet_case_color;
unsigned long RefereeUILG::bullet_case_update_time;

const char RefereeUILG::dodge_name[3] = {'d', 'g', 'e'};
char RefereeUILG::dodge_title[31];
RefereeUISKD::color_t RefereeUILG::dodge_color;
unsigned long RefereeUILG::dodge_update_time;

const char RefereeUILG::gun_indicator_name[3] = {'g', 'u', 'n'};
RefereeUISKD::UI_point RefereeUILG::gun_start_point;
RefereeUISKD::UI_point RefereeUILG::gun_end_point;

const char RefereeUILG::chassis_indicator_1_name[3] = {'c', 'h', '1'};
RefereeUISKD::UI_point RefereeUILG::chassis_1_start_point;
RefereeUISKD::UI_point RefereeUILG::chassis_1_end_point;

const char RefereeUILG::chassis_indicator_2_name[3] = {'c', 'h', '2'};
RefereeUISKD::UI_point RefereeUILG::chassis_2_start_point;
RefereeUISKD::UI_point RefereeUILG::chassis_2_end_point;

unsigned long RefereeUILG::angle_update_time;

const char RefereeUILG::main_enemy_name[3] = {'e', 'n', 'm'};
RefereeUISKD::UI_point RefereeUILG::main_enemy;

bool RefereeUILG::bullet_case_opened = false;
int RefereeUILG::remaining_bullet_count = 0;


void RefereeUILG::reset() {
    Referee::remove_all_blocking();
    reset_data_UI();
    reset_chassis_UI();
    reset_vision_UI();
}


void RefereeUILG::revise_cap(float cap_volt) {
    // if (WITHIN_RECENT_TIME(cap_update_time, 400)) return;
    Shell::snprintf(cap_title, sizeof(cap_title), "CAP  VOLT  : %.1f", cap_volt);
    RefereeUISKD::color_t title_color;
    if (cap_volt > 15.0f) {
        title_color = RefereeUISKD::GREEN;
    } else {
        title_color = RefereeUISKD::YELLOW;
    }
    RefereeUISKD::revise_character(cap_name, cap_title, title_color);
    cap_update_time = SYSTIME;
}

void RefereeUILG::set_bullet_case_state(bool bullet_case_opened_) {
    // if (WITHIN_RECENT_TIME(top_update_time, 400)) return;
    bullet_case_opened = bullet_case_opened_;
    update_bullet_case_line();
    RefereeUISKD::revise_character(bullet_case_name, bullet_case_title, bullet_case_color);
}

void RefereeUILG::set_remaining_bullet_count(int remaining_bullet_count_) {
    // if (WITHIN_RECENT_TIME(top_update_time, 400)) return;
    remaining_bullet_count = remaining_bullet_count_;
    update_bullet_case_line();
    RefereeUISKD::revise_character(bullet_case_name, bullet_case_title, bullet_case_color);
}

void RefereeUILG::update_bullet_case_line() {
    RefereeUISKD::color_t title_color;
    if (bullet_case_opened) {
        Shell::snprintf(bullet_case_title, sizeof(bullet_case_title), "CASE [R]: OPEN  %3d", remaining_bullet_count);
        bullet_case_color = RefereeUISKD::ACCENT_COLOR;
    } else {
        Shell::snprintf(bullet_case_title, sizeof(bullet_case_title), "CASE [R]: CLOSE %3d", remaining_bullet_count);
        bullet_case_color = RefereeUISKD::GREEN;
    }
    bullet_case_update_time = SYSTIME;
}

void RefereeUILG::toggle_dodge(bool dodge_stat) {
    // if (WITHIN_RECENT_TIME(dodge_update_time, 400)) return;
    RefereeUISKD::color_t title_color;
    if (dodge_stat) {
        Shell::snprintf(dodge_title, sizeof(dodge_title), "DODGE MODE : ON   [X]");
        title_color = RefereeUISKD::GREEN;
    } else {
        Shell::snprintf(dodge_title, sizeof(dodge_title), "DODGE MODE : OFF  [X]");
        title_color = RefereeUISKD::ACCENT_COLOR;
    }
    RefereeUISKD::revise_character(dodge_name, dodge_title, title_color);
    dodge_update_time = SYSTIME;
}

void RefereeUILG::set_chassis_angle(float angle) {
    // if (WITHIN_RECENT_TIME(angle_update_time, 25)) return;
    double x_offset = 25.0f * sin(angle);
    double y_offset = 25.0f * cos(angle);
    chassis_1_start_point.x = (uint32_t) (960.0f - x_offset);
    chassis_1_end_point.x = (uint32_t) (960.0f + x_offset);
    chassis_1_start_point.y = (uint32_t) (200.0f - y_offset);
    chassis_1_end_point.y = (uint32_t) (200.0f + y_offset);
//    chassis_2_end_point.x     = (uint32_t) (960.0f + 30.0f*sin(angle));
//    chassis_2_end_point.y     = (uint32_t) (200.0f + 30.0f*cos(angle));
    RefereeUISKD::revise_line(chassis_indicator_1_name, chassis_1_start_point, chassis_1_end_point);
//    RefereeUISKD::revise_line(chassis_indicator_2_name, chassis_1_start_point, chassis_2_end_point);
    angle_update_time = SYSTIME;
}

void RefereeUILG::set_main_enemy(RefereeUISKD::UI_point enemy_loc) {
    main_enemy.x = enemy_loc.x;
    main_enemy.y = enemy_loc.y;
    RefereeUISKD::revise_shape_loc(main_enemy_name, main_enemy, RefereeUISKD::ACCENT_COLOR);
}

void RefereeUILG::reset_data_UI() {
    RefereeUISKD::remove_layer(1);
    Shell::snprintf(cap_title, sizeof(cap_title), "CAP   : ?      ", 0.0f);
    RefereeUISKD::add_label(cap_name, {100, 540}, RefereeUISKD::GREEN, 1, 15, 2, cap_title);
    Shell::snprintf(dodge_title, sizeof(dodge_title), "DODGE [X]: ?    [X]");
    RefereeUISKD::add_label(dodge_name, {100, 570}, RefereeUISKD::ACCENT_COLOR, 1, 15, 2, dodge_title);
    update_bullet_case_line();
    RefereeUISKD::add_label(bullet_case_name, {100, 600}, bullet_case_color, 1, 15, 2, bullet_case_title);
}

void RefereeUILG::reset_chassis_UI() {
    RefereeUISKD::remove_layer(2);
    RefereeUILG::gun_end_point = {960, 200};
    RefereeUILG::gun_start_point = {960, 250};
    RefereeUISKD::add_line(gun_indicator_name, 2, RefereeUISKD::GREEN, gun_start_point, gun_end_point, 2);
    RefereeUILG::chassis_1_end_point = {960, 200 + 25};
    RefereeUILG::chassis_1_start_point = {960, 200 - 25};
    RefereeUISKD::add_line(chassis_indicator_1_name, 2, RefereeUISKD::ACCENT_COLOR, chassis_1_start_point,
                           chassis_1_end_point, 30);
//    RefereeUILG::chassis_2_end_point = {960, 230};
//    RefereeUISKD::add_line(chassis_indicator_2_name, 2, RefereeUISKD::ACCENT_COLOR, chassis_1_start_point, chassis_2_end_point, 20);
}

void RefereeUILG::reset_vision_UI() {
    RefereeUISKD::remove_layer(3);
    RefereeUILG::main_enemy = {960, 540};
    RefereeUISKD::add_circle(main_enemy_name, 3, RefereeUISKD::ACCENT_COLOR, main_enemy, 10, 10);
}