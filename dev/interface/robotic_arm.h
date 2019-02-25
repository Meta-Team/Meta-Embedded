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

#if defined(BOARD_RM_2018_A)
#else
#error "RoboticArm interface is only developed for RM board 2018 A."
#endif

/**
 * @name RoboticArm
 * @brief interface to handle clamp and rotation motor of robotic arm
 * @pre hardware is properly configured. Rotation motor CAN ID = 5 connect to POWER3_PH4. Clamp is connected to POWER1_PH2.
 */
class RoboticArm {

public:

    /**
     * @brief get rotation motor actual angle
     * @return accumulate angle in [degree]. Outward is negative
     */
    static float get_motor_actual_angle();

    /**
     * @brief set current angle as front angle
     */
    static void reset_front_angle();

    enum clamp_status_t {
        CLAMP_RELAX = PAL_LOW,
        CLAMP_CLAMPED = PAL_HIGH
    };

    /**
     * @brief get status of clamp
     * @return
     */
    static clamp_status_t get_clamp_status();

    /**
     * @brief perform action on clamp
     * @param target_status
     */
    static void clamp_action(clamp_status_t target_status);

    /**
     * @brief set rotation motor target current and store in this interface
     * @param target_current
     */
    static void set_motor_target_current(int target_current);

    /**
     * @brief send rotation motor target current
     * @return
     * @note the reason to separate set and send function is that C620 can only holds the targets for a
     *       while, so it's needed to send repreatly in a thread
     */
    static bool send_motor_target_current();

    /**
     * @brief initialize this interface with properly started CANInterface
     * @param can_interface pointer to properly started CANInterface
     */
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

    static constexpr float  motor_decelerate_ratio = 19.2f; // 3591/187 on the data sheet

};

#endif //META_INFANTRY_FETCH_BULLET_H
