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

    // TODO: exdend the interface to support velocity measurement

    typedef struct motor {

        motor_id_t id;
        bool enabled;

        int target_current;

        uint16_t front_angle_raw;

        float actual_angle;
        int round_count;

        int actual_current;

    } motor_t;

    static motor_t yaw;
    static motor_t pitch;

    static bool send_gimbal_currents();

    GimbalInterface() {
        yaw.id = YAW_ID;
        yaw.enabled = false;
        pitch.id = PIT_ID;
        pitch.enabled = false;
    }

    static void set_can_interface (CANInterface* can_interface) {
        can = can_interface;
    }

    static bool process_motor_feedback (CANRxFrame *rxmsg);

private:

    static CANInterface* can;

};


#endif //META_INFANTRY_GIMBAL_H
