//
// Created by kerui on 2021/7/29.
//

#include "new_sentry_chassis_logic.h"
#include "new_sentry_chassis_scheduler.h"

SChassisLG::mode_t SChassisLG::mode_ = FORCED_RELAX_MODE;
float SChassisLG::target_dest = 0;
SChassisLG::DirectionSwitchThread SChassisLG::directionSwitchThread;

void SChassisLG::init(tprio_t direction_switch_thd_prio) {
    directionSwitchThread.start(direction_switch_thd_prio);
}

void SChassisLG::set_mode(mode_t mode) {
    if (mode_ == mode) return;
    mode_ = mode;
    if (mode_ == FORCED_RELAX_MODE) {
        SChassisSKD::set_mode(SChassisSKD::FORCED_RELAX_MODE);
    } else if (mode_ == MANUAL_MODE) {
        SChassisSKD::set_mode(SChassisSKD::ENABLED);
    }else if (mode_ == SHUTTLE_MODE) {
        SChassisSKD::set_mode(SChassisSKD::ENABLED);
        SChassisSKD::set_target(60.f);
    }
}

void SChassisLG::set_dest(float dest) {
    target_dest = dest;
}

float SChassisLG::get_dest() {
    return target_dest;
}

void SChassisLG::DirectionSwitchThread::main() {
    setName("SChassisLGDirectionSwitch");
    while (!shouldTerminate()) {
        if (mode_ != FORCED_RELAX_MODE) {
            if (mode_ == SHUTTLE_MODE) {
                if (SChassisSKD::get_location(SChassisBase::R) > 50) {
                    target_dest = -60.f;
                } else if (SChassisSKD::get_location(SChassisBase::R) < -50) {
                    target_dest = 60.f;
                }
            }
            SChassisSKD::set_target(target_dest);
        }

        sleep(TIME_MS2I(DIRECTION_INSPECTION_INTERVAL));
    }
}