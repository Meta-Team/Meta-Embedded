//
// Created by liuzikai on 2018-12-29.
// Zhu Kerui wrote code about processing gimbal feedback.
// Feng Chuhao wrote code about sending gimbal currents.
//

#ifndef META_INFANTRY_GIMBAL_H
#define META_INFANTRY_GIMBAL_H

#include "ch.hpp"
#include "hal.h"
#include "can_interface.h"

#define GIMBAL_INTERFACE_ENABLE_CLIP                  TRUE

#if GIMBAL_INTERFACE_ENABLE_CLIP
#define GIMBAL_INTERFACE_MAX_CURRENT 5000
#endif


class GimbalInterface {

public:

    typedef enum {
        YAW_ID = 0,
        PIT_ID = 1
    } motor_id_t;

    /** Motor Interface **/
    typedef struct motor {

        motor_id_t id;
        bool enabled;  // if not enabled, 0 current will be sent in send_gimbal_currents

        int target_current;

        uint16_t front_angle_raw;

        uint16_t actual_angle_raw;

        /* Normalized Angle and Rounds
         *  Using the front angle_raw as reference.
         *  Range: -180.0 () to 180.0 ()
         */
        // TODO: test the sign of clockwise and counter-clockwise
        float actual_angle;
        int round_count;

        time_msecs_t sample_time;  // last sample time, for velocity calculation

        float angular_velocity;  // instant angular velocity [degree/ms]

        int actual_current;  // feedback current

    } motor_t;

    static motor_t yaw;
    static motor_t pitch;

    /**
     * @brief send target_current of each motor
     * @return
     */
    static bool send_gimbal_currents();

    /**
     * @brief process CAN rx frame
     * @param rxmsg
     * @return whether the rx frame is from gimbal motors
     */
    static bool process_motor_feedback (CANRxFrame *rxmsg);

    /**
     * Default constructor
     */
    GimbalInterface() {
        yaw.id = YAW_ID;
        yaw.enabled = false;
        yaw.sample_time = 0;
        pitch.id = PIT_ID;
        pitch.enabled = false;
        pitch.sample_time = 0;
    }

    /**
     * @brief set the CAN interface
     * @param can_interface
     */
    static void set_can_interface (CANInterface* can_interface) {
        can = can_interface;
    }


private:

    static CANInterface* can;

};


#endif //META_INFANTRY_GIMBAL_H
