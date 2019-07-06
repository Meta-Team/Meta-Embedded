//
// Created by liuzikai on 2019-06-15.
//

/**
 * @file    chassis_logic.cpp
 * @brief   Generate target values for ChassisSKD. Support follow-gimbal mode and dodge mode.
 *
 * @addtogroup chassis
 * @{
 */

#include "chassis_logic.h"

ChassisLG::action_t ChassisLG::action = FORCED_RELAX_MODE;
float ChassisLG::target_vx;
float ChassisLG::target_vy;
float ChassisLG::target_theta;

tprio_t ChassisLG::dodge_thread_prio;
ChassisLG::DodgeModeSwitchThread ChassisLG::dodgeModeSwitchThread;
chibios_rt::ThreadReference ChassisLG::dodgeThreadReference;


void ChassisLG::init(tprio_t dodge_thread_prio_) {
    dodge_thread_prio = dodge_thread_prio_;
    dodgeModeSwitchThread.started = true;
    dodgeThreadReference = dodgeModeSwitchThread.start(dodge_thread_prio);
}

ChassisLG::action_t ChassisLG::get_action() {
    return action;
}

void ChassisLG::set_action(ChassisLG::action_t value) {
    if (value == action) return;  // avoid repeating setting target_theta, etc.

    action = value;
    if (action == FORCED_RELAX_MODE) {
        ChassisSKD::set_mode(ChassisSKD::FORCED_RELAX_MODE);
    } else if (action == FOLLOW_MODE) {
        ChassisSKD::set_mode(ChassisSKD::GIMBAL_COORDINATE_MODE);
        apply_target();
    } else if (action == DODGE_MODE) {
        ChassisSKD::set_mode(ChassisSKD::GIMBAL_COORDINATE_MODE);
        target_theta = DODGE_MODE_THETA;
        chSysLock();
        if (!dodgeModeSwitchThread.started) {
            dodgeModeSwitchThread.started = true;
            chSchWakeupS(dodgeThreadReference.getInner(), 0);
        }
        chSysUnlock();
    }
}

void ChassisLG::set_target(float vx, float vy) {
    target_vx = vx;
    target_vy = vy;
    if (action == FOLLOW_MODE) {
        target_theta = 0;
    }
    // For DODGE_MODE keep current target_theta unchanged
    apply_target();
}

void ChassisLG::apply_target() {
    ChassisSKD::set_target(target_vx, target_vy, target_theta);
}

void ChassisLG::DodgeModeSwitchThread::main() {

    setName("Chassis_Dodge");
    while(!shouldTerminate()) {

        chSysLock();  /// ---------------------------------- Enter Critical Zone ----------------------------------
        if (action != DODGE_MODE) {
            started = false;
            chSchGoSleepS(CH_STATE_SUSPENDED);
        }
        chSysUnlock();  /// ---------------------------------- Exit Critical Zone ----------------------------------

        if (!ABS_IN_RANGE(ChassisSKD::get_actual_theta() - (-target_theta), 20)) {
            target_theta = -target_theta;
        }
        /**
         * If next target_theta is too close to current theta (may due to gimbal rotation), do not switch target to
         * create large difference to avoid pause
         */

        apply_target();

        sleep(TIME_MS2I(DODGE_MODE_SWITCH_INTERVAL));
    }
}

/** @} */