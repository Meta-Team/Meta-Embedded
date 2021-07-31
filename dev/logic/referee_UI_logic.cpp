//
// Created by 钱晨 on 7/19/21.
//

#include "referee_UI_logic.h"
#include "shell.h"
#include "common_macro.h"
#include "super_capacitor_port.h"
#include "chassis_logic.h"
#include "shoot_logic.h"
#include "chassis_scheduler.h"
#include "vision_scheduler.h"

const char RefereeUILG::cap_name[3] = {'c', 'a', 'p'};
char RefereeUILG::cap_title[64];
RefereeUISKD::color_t RefereeUILG::cap_volt_line_color;
unsigned long RefereeUILG::cap_update_time;

const char RefereeUILG::bullet_case_name[3] = {'t', 'o', 'p'};
char RefereeUILG::bullet_case_title[64];
RefereeUISKD::color_t RefereeUILG::bullet_case_color;
unsigned long RefereeUILG::bullet_case_update_time;

const char RefereeUILG::dodge_name[3] = {'d', 'g', 'e'};
char RefereeUILG::dodge_title[64];
RefereeUISKD::color_t RefereeUILG::dodge_line_color;
unsigned long RefereeUILG::dodge_update_time;

const char RefereeUILG::gun_indicator_name[3] = {'g', 'u', 'n'};
RefereeUISKD::ui_point_t RefereeUILG::gun_start_point;
RefereeUISKD::ui_point_t RefereeUILG::gun_end_point;

const char RefereeUILG::chassis_indicator_1_name[3] = {'c', 'h', '1'};
RefereeUISKD::ui_point_t RefereeUILG::chassis_1_start_point;
RefereeUISKD::ui_point_t RefereeUILG::chassis_1_end_point;

const char RefereeUILG::chassis_indicator_2_name[3] = {'c', 'h', '2'};
RefereeUISKD::ui_point_t RefereeUILG::chassis_2_start_point;
RefereeUISKD::ui_point_t RefereeUILG::chassis_2_end_point;

unsigned long RefereeUILG::angle_update_time;

const char RefereeUILG::main_enemy_name[3] = {'e', 'n', 'm'};
RefereeUISKD::ui_point_t RefereeUILG::main_enemy;
RefereeUISKD::color_t RefereeUILG::main_enemy_line_color;

const char RefereeUILG::vision_name[3] = {'v', 'i', 's'};
char RefereeUILG::vision_title[64];
int RefereeUILG::vision_displayed_bullet_speed = 0;

int RefereeUILG::cap_volt = 0;
bool RefereeUILG::bullet_case_opened = false;
bool RefereeUILG::dodge_mode_enabled = false;
int RefereeUILG::remaining_bullet_count = 0;
float RefereeUILG::current_chassis_angle = 0;

RefereeUILG::DataFetchThread RefereeUILG::data_fetch_thread;

void RefereeUILG::start(tprio_t fetch_thread_prio) {
    reset();
    data_fetch_thread.start(fetch_thread_prio);
}

void RefereeUILG::reset() {
    RefereeUISKD::remove_all();
    reset_data_UI();
    reset_chassis_UI();
    reset_vision_UI();
}

void RefereeUILG::set_cap_volt(float cap_volt_) {
    if (cap_volt /* 0.1V */ == (int) (cap_volt_ * 10)) return;
    // if (WITHIN_RECENT_TIME(cap_update_time, 400)) return;
    cap_volt = (int) (cap_volt_ * 10);
    update_cap_volt_line();
    RefereeUISKD::revise_label(cap_name, cap_title, cap_volt_line_color);
}

void RefereeUILG::update_cap_volt_line() {
    Shell::snprintf(cap_title, sizeof(cap_title), "CAP   : %.1f", cap_volt / 10.0f);
    if (cap_volt > 15.0f) {
        cap_volt_line_color = RefereeUISKD::GREEN;
    } else {
        cap_volt_line_color = RefereeUISKD::YELLOW;
    }
    cap_update_time = SYSTIME;
}

void RefereeUILG::set_bullet_case_state(bool bullet_case_opened_) {
    // if (WITHIN_RECENT_TIME(top_update_time, 400)) return;
    if (bullet_case_opened == bullet_case_opened_) return;
    bullet_case_opened = bullet_case_opened_;
    update_bullet_case_line();
    RefereeUISKD::revise_label(bullet_case_name, bullet_case_title, bullet_case_color);
}

void RefereeUILG::set_remaining_bullet_count(int remaining_bullet_count_) {
    // if (WITHIN_RECENT_TIME(top_update_time, 400)) return;
    if (remaining_bullet_count == remaining_bullet_count_) return;
    remaining_bullet_count = remaining_bullet_count_;
    update_bullet_case_line();
    RefereeUISKD::revise_label(bullet_case_name, bullet_case_title, bullet_case_color);
}

void RefereeUILG::update_bullet_case_line() {
    if (bullet_case_opened) {
        Shell::snprintf(bullet_case_title, sizeof(bullet_case_title), "BULLET: %-3d", remaining_bullet_count);
        bullet_case_color = RefereeUISKD::ACCENT_COLOR;
    } else {
        Shell::snprintf(bullet_case_title, sizeof(bullet_case_title), "BULLET: %-3d", remaining_bullet_count);
        bullet_case_color = RefereeUISKD::GREEN;
    }
    bullet_case_update_time = SYSTIME;
}

void RefereeUILG::set_dodge_state(bool dodge_mode_enabled_) {
    // if (WITHIN_RECENT_TIME(dodge_update_time, 400)) return;
    if (dodge_mode_enabled == dodge_mode_enabled_) return;
    dodge_mode_enabled = dodge_mode_enabled_;
    update_dodge_line();
    RefereeUISKD::revise_label(dodge_name, dodge_title, dodge_line_color);
}

void RefereeUILG::update_dodge_line() {
    if (dodge_mode_enabled) {
        Shell::snprintf(dodge_title, sizeof(dodge_title), "DODGE : ON");
        dodge_line_color = RefereeUISKD::GREEN;
    } else {
        Shell::snprintf(dodge_title, sizeof(dodge_title), "DODGE : OFF");
        dodge_line_color = RefereeUISKD::ACCENT_COLOR;
    }
    dodge_update_time = SYSTIME;
}

void RefereeUILG::set_chassis_angle(float angle) {
    // if (WITHIN_RECENT_TIME(angle_update_time, 25)) return;
    if (current_chassis_angle == angle) return;
    current_chassis_angle = angle;
    update_chassis_shape();
    RefereeUISKD::revise_line(chassis_indicator_1_name, chassis_1_start_point, chassis_1_end_point);
//    RefereeUISKD::revise_line(chassis_indicator_2_name, chassis_1_start_point, chassis_2_end_point);
}

void RefereeUILG::update_chassis_shape() {
    double x_offset = 25.0f * sin(current_chassis_angle);
    double y_offset = 25.0f * cos(current_chassis_angle);
    chassis_1_start_point.x = (uint32_t) (960.0f - x_offset);
    chassis_1_end_point.x = (uint32_t) (960.0f + x_offset);
    chassis_1_start_point.y = (uint32_t) (200.0f - y_offset);
    chassis_1_end_point.y = (uint32_t) (200.0f + y_offset);
//    chassis_2_end_point.x     = (uint32_t) (960.0f + 30.0f*sin(current_chassis_angle));
//    chassis_2_end_point.y     = (uint32_t) (200.0f + 30.0f*cos(current_chassis_angle));
    angle_update_time = SYSTIME;
}

void RefereeUILG::set_main_enemy(RefereeUISKD::ui_point_t enemy_loc) {
    if (main_enemy.x == enemy_loc.x && main_enemy.y == enemy_loc.y) return;
    main_enemy.x = enemy_loc.x;
    main_enemy.y = enemy_loc.y;
    RefereeUISKD::revise_shape_loc(main_enemy_name, main_enemy, RefereeUISKD::YELLOW);
}

void RefereeUILG::set_vision_bullet_speed(float speed) {
    if (vision_displayed_bullet_speed == (int) (speed * 1000)) return;
    vision_displayed_bullet_speed = (int) (speed * 1000);
    update_vision_line();
    RefereeUISKD::revise_label(vision_name, vision_title, RefereeUISKD::GREEN);
}

void RefereeUILG::update_vision_line() {
    Shell::snprintf(vision_title, sizeof(vision_title), "VIS   : %d", vision_displayed_bullet_speed);
}

void RefereeUILG::reset_data_UI() {
    //RefereeUISKD::remove_layer(1);
    chThdSleepMilliseconds(1000);
    update_vision_line();
    RefereeUISKD::add_label(vision_name, {50, 540}, RefereeUISKD::GREEN, 1, 15, 2, vision_title);
    update_cap_volt_line();
    RefereeUISKD::add_label(cap_name, {50, 570}, cap_volt_line_color, 1, 15, 2, cap_title);
    update_dodge_line();
    RefereeUISKD::add_label(dodge_name, {50, 600}, dodge_line_color, 1, 15, 2, dodge_title);
//    update_bullet_case_line();
//    RefereeUISKD::add_label(bullet_case_name, {50, 630}, bullet_case_color, 1, 15, 2, bullet_case_title);

}

void RefereeUILG::reset_chassis_UI() {
    //RefereeUISKD::remove_layer(2);
    chThdSleepMilliseconds(1000);
    RefereeUILG::gun_end_point = {960, 200};
    RefereeUILG::gun_start_point = {960, 250};
    RefereeUISKD::add_line(gun_indicator_name, 2, RefereeUISKD::GREEN, gun_start_point, gun_end_point, 2);
    update_chassis_shape();
    RefereeUISKD::add_line(chassis_indicator_1_name, 2, RefereeUISKD::ACCENT_COLOR, chassis_1_start_point,
                           chassis_1_end_point, 30);
//    RefereeUILG::chassis_2_end_point = {960, 230};
//    RefereeUISKD::add_line(chassis_indicator_2_name, 2, RefereeUISKD::ACCENT_COLOR, chassis_1_start_point, chassis_2_end_point, 20);
}

void RefereeUILG::reset_vision_UI() {
    //RefereeUISKD::remove_layer(3);
    chThdSleepMilliseconds(1000);
    RefereeUILG::main_enemy = {9999, 9999};
    RefereeUISKD::add_circle(main_enemy_name, 3, RefereeUISKD::YELLOW, main_enemy, 5, 10);
}

void RefereeUILG::DataFetchThread::main() {
    setName("RefereeUILG");
    while (!shouldTerminate()) {
        set_cap_volt(SuperCapacitor::feedback->capacitor_voltage);
        set_dodge_state(ChassisLG::get_action() == ChassisLG::DODGE_MODE);
//        set_bullet_case_state(false);  // TODO
//        set_remaining_bullet_count(ShootLG::get_remaining_bullet_count());
        set_chassis_angle(ChassisSKD::get_actual_theta() / 180.0f * PI);
        set_vision_bullet_speed(Vision::get_bullet_speed());
        {
            uint32_t x = 9999, y = 9999;  // out side the screen to hide
            if (Vision::is_detected()) Vision::get_user_view_points(x, y);
            set_main_enemy({x, y});
        }
        sleep(TIME_MS2I(100));
    }
}
