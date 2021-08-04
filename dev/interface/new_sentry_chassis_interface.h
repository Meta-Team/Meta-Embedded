//
// Created by kerui on 2021/7/29.
//

#ifndef META_INFANTRY_SENTRY_CHASSIS_INTERFACE_H
#define META_INFANTRY_SENTRY_CHASSIS_INTERFACE_H

#include "ch.hpp"
#include "hal.h"
#include "can_interface.h"
#include "motor_interface.h"
#include "common_macro.h"

class SChassisBase {
public:
// Start at the front right, goes in CCW
enum motor_id_t {
    R, // right motor, 0
    L, // left motor, 1
    MOTOR_COUNT
};
};
class SChassisIF : public SChassisBase, public MotorIFBase {

public:

    static void init(CANInterface* can1_interface, CANInterface *can2_interface,
                     motor_can_config_t motor_can_config[MOTOR_COUNT]);

    static CANInterface::motor_feedback_t *feedback[MOTOR_COUNT];

    static int *target_current[MOTOR_COUNT];

    static void clip_chassis_current();

private:

    static CANInterface* can1_;
    static CANInterface* can2_;
};


#endif //META_INFANTRY_SENTRY_CHASSIS_INTERFACE_H
