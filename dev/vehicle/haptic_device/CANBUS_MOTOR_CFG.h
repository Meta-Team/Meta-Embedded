//
// Created by 钱晨 on 10/29/21.
//

#ifndef META_INFANTRY_CANBUS_MOTOR_CFG_H
#define META_INFANTRY_CANBUS_MOTOR_CFG_H

#include "can_motor_feedback.h"

class CANBUS_MOTOR_CFG {
public:
    enum motor_id_t {
        YAW,
        MOTOR_COUNT
    };
    static CANMotorBase CANMotorProfile[motor_id_t::MOTOR_COUNT];
};






#endif //META_INFANTRY_CANBUS_MOTOR_CFG_H
