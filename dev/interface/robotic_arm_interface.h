//
// Created by Ye Anbang on 2019/2/2.
// Revised by Zhu Kerui on 2019/7/11.
//

#ifndef META_INFANTRY_FETCH_BULLET_H
#define META_INFANTRY_FETCH_BULLET_H

#include "ch.hpp"
#include "hal.h"
#include "hal_pal.h"
#include "can_interface.h"

#if defined(BOARD_RM_2018_A)
#else
#error "RoboticArm interface is only developed for RM board 2018 A."
#endif

/**
 * @name RoboticArm
 * @brief interface to handle rotation motor of robotic arm
 * @pre hardware is properly configured. Rotation motor CAN ID = 5 connect to POWER3_PH4. Clamp is connected to POWER1_PH2.
 */
class RoboticArmIF {

public:

    static float present_angle;

    static float present_velocity;

    static time_msecs_t motor_last_update_time;

    static int16_t motor_target_current;

    /**
     * @brief initialize this interface with properly started CANInterface
     * @param can_interface pointer to properly started CANInterface
     * @param motor_front_angle_raw   the raw angle of motor when the robotic arm is inside
     */
    static void init(CANInterface *can_interface);

    /**
     * @brief send rotation motor target current
     * @return
     * @note the reason to separate set and send function is that C620 can only holds the targets for a
     *       while, so it's needed to send repeatedly in a thread
     */
    static bool send_current();

private:

    static uint16_t motor_last_actual_angle_raw;

    static void process_feedback(CANRxFrame const *rxmsg);

    static CANInterface *can;

    friend class CANInterface;

private:

    /** Configurations **/

    static constexpr float  motor_decelerate_ratio = 19.2f; // 3591/187 on the data sheet

};

#endif //META_INFANTRY_FETCH_BULLET_H
