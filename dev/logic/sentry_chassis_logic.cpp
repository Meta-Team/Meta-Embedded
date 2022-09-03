//
// Created by Quoke on 8/2/2022.
//

#include "sentry_chassis_logic.h"

SChassisLG::mode_t SChassisLG::mode = FORCED_RELAX_MODE;
SChassisLG::MotionControlThread SChassisLG::motion_control_thread;

float SChassisLG::target_velocity = 0.0f;
float SChassisLG::set_destination = 0.0f;
float SChassisLG::positions[] = {100,0,100
                                 ,100};
int SChassisLG::iterator = 0;

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

float SChassisLG::set_random_destination(){
    float destination;
    std::random_device rnd;
    std::mt19937 random(rnd());
    std::uniform_real_distribution<float> distribution(0, 100);
    destination = distribution(random);
    return destination;
}

void SChassisLG::set_scalar_destination(){
    iterator++;
    if(iterator>3){
        iterator = 0;
    }
    set_destination = positions[iterator];
}

void SChassisLG::set_velocity(float target_velocity_) {
    SChassisLG::target_velocity = target_velocity_;
}

void SChassisLG::MotionControlThread::main() {
    setName("sentry_motion_control_thread");
    while (!shouldTerminate()) {
        chSysLock();
        switch (mode) {
            case FORCED_RELAX_MODE:
                // Settings in set_mode function will automatically execute emergency stop.
                break;
            case AUTO_MODE: {
                // If reaches to the destination, pick a new one randomly.
//                if(ABS_IN_RANGE(SChassisSKD::present_position() - set_destination, 10.0f)) {
//                    set_destination = set_random_destination(set_destination);
//                    SChassisSKD::set_destination(set_destination);
//                }
                if(ABS_IN_RANGE(SChassisSKD::present_position() - set_destination, 100.0f)) {
                    set_destination = 50000 - set_destination;
                    SChassisSKD::set_destination(set_destination);
                }
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
        LOG("%f",set_destination);
    }
}