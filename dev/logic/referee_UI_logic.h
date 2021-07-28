//
// Created by 钱晨 on 7/19/21.
//

#ifndef META_INFANTRY_REFEREE_UI_LOGIC_H
#define META_INFANTRY_REFEREE_UI_LOGIC_H

#include "referee_UI_update_scheduler.h"
#include "ch.hpp"
#include "hal.h"
#include "math.h"
using namespace chibios_rt;

class RefereeUILG {
public:
    static void reset();
    static void revise_cap(float cap_volt);
    static void toggle_top(bool top_stat);
    static void toggle_dodge(bool dodge_stat);
    static void set_chassis_angle(float angle);
    static void set_main_enemy(RefereeUISKD::UI_point enemy_loc);
private:
    static const char cap_name[3];
    static char cap_title[31];
    static unsigned long cap_update_time;

    static const char top_name[3];
    static char top_title[31];
    static unsigned long top_update_time;

    static const char dodge_name[3];
    static char dodge_title[31];
    static unsigned long dodge_update_time;

    static const char chassis_indicator_1_name[3];
    static RefereeUISKD::UI_point chassis_1_start_point;
    static RefereeUISKD::UI_point chassis_1_end_point;

    static const char chassis_indicator_2_name[3];
    static RefereeUISKD::UI_point chassis_2_start_point;
    static RefereeUISKD::UI_point chassis_2_end_point;

    static const char gun_indicator_name[3];
    static RefereeUISKD::UI_point gun_start_point;
    static RefereeUISKD::UI_point gun_end_point;

    static unsigned long angle_update_time;

    static const char main_enemy_name[3];
    static RefereeUISKD::UI_point enemy_info[5];
    static RefereeUISKD::UI_point main_enemy;

    static void reset_data_UI();
    static void reset_chassis_UI();
    static void reset_vision_UI();

};


#endif //META_INFANTRY_REFEREE_UI_LOGIC_H
