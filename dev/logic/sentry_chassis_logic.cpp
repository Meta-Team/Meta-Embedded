//
// Created by Quoke on 8/2/2022.
//

#include "sentry_chassis_logic.h"

SChassisLG::mode_t SChassisLG::mode = FORCED_RELAX_MODE;
SChassisLG::MotionControlThread SChassisLG::motion_control_thread;

float SChassisLG::target_velocity = 0.0f;

void SChassisLG::init(tprio_t mtn_ctl_prio) {
    motion_control_thread.start(mtn_ctl_prio);
}

void SChassisLG::set_mode(SChassisLG::mode_t mode_) {
    mode = mode_;
    switch (mode_) {
        case FORCED_RELAX_MODE:
            SChassisSKD::set_mode(SChassisSKD::FORCED_RELAX_MODE);
            break;
        case AUTO_MODE:
            SChassisSKD::set_mode(SChassisSKD::POSITION_MODE);
            break;
        case MANUAL_MODE:
            SChassisSKD::set_mode(SChassisSKD::VELOCITY_MODE);
            break;
    }

}

void SChassisLG::set_velocity(float target_velocity_) {
    SChassisLG::target_velocity = target_velocity_;
}

void SChassisLG::MotionControlThread::main() {
    setName("sentry_motion_control_thread");
    // Uniform random number generation.
//    std::random_device rnd;
//    std::mt19937 random(rnd());
//    std::uniform_real_distribution<float> distribution(0, 300);

    // Random sentry chassis destination
    float destination = 0.0f;
    while (!shouldTerminate()) {
        chSysLock();
        switch (mode) {
            case FORCED_RELAX_MODE:
                // Settings in set_mode function will automatically execute emergency stop.
                break;
            case AUTO_MODE: {
                // If reaches to the destination, pick a new one randomly.
//                if(ABS_IN_RANGE(SChassisSKD::present_position() - destination, 10.0f)) {
//                    destination = distribution(random);
//                    SChassisSKD::set_destination(destination);
//                }
                break;
            }
            case MANUAL_MODE: {
                // Higher level program will call SChassisLG::set_velocity to set the target velocity.
                SChassisSKD::set_velocity(target_velocity);
                break;
            }
        }
        chSysUnlock();
        sleep(TIME_MS2I(SCHASSIS_MTN_CTL_INTERVAL));
    }
}