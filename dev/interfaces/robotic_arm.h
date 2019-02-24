//
// Created by Ye Anbang on 2019/2/2.
//

#ifndef META_INFANTRY_FETCH_BULLET_H
#define META_INFANTRY_FETCH_BULLET_H

#include "ch.hpp"
#include "hal.h"
#include "hal_pal.h"
#include "can_interface.h"
#include "port_to_string.h"

static uint16_t rotate_current = 30;  //TODO change it
static uint16_t release_current = 30;  //TODO change it
static uint16_t inactive_current = 30;  //TODO: change it

// TODO: make sure that there is no CAN conflicts
/**
 * @pre rotation motor CAN ID = 5
 * @pre clamp connect to PH2
 */
class RoboticArm {

public:

    static float get_motor_actual_angle();

    static void reset_front_angle();

    enum clamp_status_t {
        CLAMP_RELAX = PAL_LOW,
        CLAMP_CLAMPED = PAL_HIGH
    };

    static clamp_status_t get_clamp_status();

    static void clamp_action(clamp_status_t target_status);

    static void set_motor_target_current(int target_current);

    static bool send_motor_target_current();

    static void init(CANInterface *can_interface);

private:

    static clamp_status_t _clamp_status; // local storage

    static float motor_accumulate_angle;
    static uint16_t motor_last_actual_angle_raw;

    static int motor_target_current;

    static void process_motor_feedback(CANRxFrame const *rxmsg);

    static CANInterface *can;

    friend CANInterface;

private:

    /** Configurations **/
    static float constexpr motor_decelerate_ratio = 19.2f; // 3591/187 on the data sheet

};

#endif //META_INFANTRY_FETCH_BULLET_H
