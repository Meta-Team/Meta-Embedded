//
// Created by 钱晨 on 10/29/21.
//

#include "haptic_scheduler.h"

haptic_scheduler::skdThread haptic_scheduler::SKDThread;
PIDController haptic_scheduler::pidController[can_motor_interface::MOTOR_COUNT];

void haptic_scheduler::start(tprio_t SKD_PRIO) {
    SKDThread.start(SKD_PRIO);
}

void haptic_scheduler::skdThread::main() {
    setName("HapticSKDThread");
    float target = 500.0f;
    int t = SYSTIME;
    pidController[0].change_parameters({0.01,0.001,0.0001,100.0,8000.0});
    while(!shouldTerminate()) {
        int output = (int)pidController[0].calc(can_motor_interface::motor_feedback[0].actual_velocity, target);
        if(ABS_IN_RANGE(output, 100)) {
            LED::led_on(4);
        } else {
            LED::led_off(4);
        }
        can_motor_interface::set_current(can_motor_interface::YAW, output);
        can_motor_interface::post_target_current(CANMotorBase::can_channel_1, 0x1FF);
        if(!WITHIN_RECENT_TIME(t, 1000)) {
            t = SYSTIME;
            target = -target;
        }
        sleep(TIME_MS2I(5));
    }
}
