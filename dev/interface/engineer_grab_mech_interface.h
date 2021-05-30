//
// Created by Qian Chen on 3/23/21.
//

#ifndef META_INFANTRY_ENGINEER_GRAB_MECH_INTERFACE_H
#define META_INFANTRY_ENGINEER_GRAB_MECH_INTERFACE_H

#include "hal.h"
#include "can_interface.h"

/// Board Guard
#if defined(BOARD_RM_2018_A)
#elif defined(BOARD_RM_2017)
#else
#error "engineerGrabMechIF has not been defined for selected board"
#endif

class EngGrabMechBase {
public:
    enum motor_id_t {
        BELT_L = 0,
        BELT_R = 1,
        ROTATION_HAND = 2,
        GRABER_L = 3,
        GRABER_R = 4,
        MOTOR_COUNT = 5
    };
};
class EngGrabMechIF: public EngGrabMechBase{
public:
    enum motor_can_channel_t {
        none_can_channel,
        can_channel_1,
        can_channel_2
    };

    struct motor_can_config_t {
        motor_can_channel_t motor_can_channel;
        unsigned motor_can_id;
        CANInterface::motor_type_t motor_type;
    };

    static void init( CANInterface *can1_interface, CANInterface *can2_interface,
                      motor_can_config_t motor_can_config[MOTOR_COUNT]);

    static CANInterface::motor_feedback_t *feedback[MOTOR_COUNT];

    static int *target_current[MOTOR_COUNT];

private:

    static CANInterface *can1_;
    static CANInterface *can2_;

    friend CANInterface;
};


#endif //META_INFANTRY_ENGINEER_GRAB_MECH_INTERFACE_H
