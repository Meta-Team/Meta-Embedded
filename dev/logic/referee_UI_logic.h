//
// Created by 钱晨 on 7/19/21.
//

#ifndef META_INFANTRY_REFEREE_UI_LOGIC_H
#define META_INFANTRY_REFEREE_UI_LOGIC_H

#include "referee_UI_update_scheduler.h"
#include "ch.hpp"
#include "hal.h"
#include "math.h"

class RefereeUILG {
public:
    static void start(tprio_t fetch_thread_prio);
    static void reset();
private:

    static void set_cap_volt(float cap_volt_);
    static void set_bullet_case_state(bool bullet_case_opened);
    static void set_remaining_bullet_count(int remaining_bullet_count_);
    static void set_dodge_state(bool dodge_stat);
    static void set_chassis_angle(float angle);  // rad
    static void set_main_enemy(RefereeUISKD::ui_point_t enemy_loc);
    static void set_vision_bullet_speed(float speed);

    static const char cap_name[3];
    static char cap_title[64];
    static RefereeUISKD::color_t cap_volt_line_color;
    static unsigned long cap_update_time;

    static const char bullet_case_name[3];
    static char bullet_case_title[64];
    static RefereeUISKD::color_t bullet_case_color;
    static unsigned long bullet_case_update_time;

    static const char dodge_name[3];
    static char dodge_title[64];
    static RefereeUISKD::color_t dodge_line_color;
    static unsigned long dodge_update_time;

    static const char chassis_indicator_1_name[3];
    static RefereeUISKD::ui_point_t chassis_1_start_point;
    static RefereeUISKD::ui_point_t chassis_1_end_point;

    static const char chassis_indicator_2_name[3];
    static RefereeUISKD::ui_point_t chassis_2_start_point;
    static RefereeUISKD::ui_point_t chassis_2_end_point;

    static const char gun_indicator_name[3];
    static RefereeUISKD::ui_point_t gun_start_point;
    static RefereeUISKD::ui_point_t gun_end_point;

    static unsigned long angle_update_time;

    static const char main_enemy_name[3];
    static RefereeUISKD::ui_point_t main_enemy;
    static RefereeUISKD::color_t main_enemy_line_color;

    static const char vision_name[3];
    static char vision_title[64];
    static int vision_displayed_bullet_speed;  // [mm/s]

    static int cap_volt;  // [0.1V]
    static bool bullet_case_opened;
    static bool dodge_mode_enabled;
    static int remaining_bullet_count;
    static float current_chassis_angle;

    static void reset_data_UI();
    static void reset_chassis_UI();
    static void reset_vision_UI();

    static void update_bullet_case_line();
    static void update_cap_volt_line();
    static void update_dodge_line();
    static void update_vision_line();
    static void update_chassis_shape();

    class DataFetchThread: public chibios_rt::BaseStaticThread<1024> {
        void main() override;
    };

    static DataFetchThread data_fetch_thread;

};


#endif //META_INFANTRY_REFEREE_UI_LOGIC_H
