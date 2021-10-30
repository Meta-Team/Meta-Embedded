//
// Created by 钱晨 on 10/29/21.
//

#include "haptic_scheduler.h"

haptic_scheduler::skdThread haptic_scheduler::SKDThread;
PIDController haptic_scheduler::v2iController[can_motor_interface::MOTOR_COUNT];
PIDController haptic_scheduler::a2vController[can_motor_interface::MOTOR_COUNT];

void haptic_scheduler::start(tprio_t SKD_PRIO) {
    SKDThread.start(SKD_PRIO);
}

void haptic_scheduler::skdThread::main() {
    setName("HapticSKDThread");
    float target = 0.0f;
    float targetV = 0.0f;
    int t = SYSTIME;
    v2iController[0].change_parameters({0.013, 0.001, 0.0001, 1000.0, 8000.0});
    a2vController[0].change_parameters({150, 0.0f, 0, 500, 3000});
    while(!shouldTerminate()) {
        targetV = v2iController[0].calc(can_motor_interface::motor_feedback[0].accumulate_angle(), target);
        int output = (int)v2iController[0].calc(can_motor_interface::motor_feedback[0].actual_velocity, targetV);
        can_motor_interface::set_current(can_motor_interface::YAW, output);
        can_motor_interface::post_target_current(CANMotorBase::can_channel_1, 0x1FF);
        if(!WITHIN_RECENT_TIME(t, 10000)) {
            t = SYSTIME;
            target = -target;
        }
        sleep(TIME_MS2I(1));
    }
}
