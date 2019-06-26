//
// Created by liuzikai on 2019-06-15.
//

#include "chassis_logic.h"

ChassisLG::action_t ChassisLG::action = STOP_MODE;
float ChassisLG::target_vx;
float ChassisLG::target_vy;
float ChassisLG::target_theta;

tprio_t ChassisLG::dodge_thread_prio;
ChassisLG::DodgeModeSwitchThread ChassisLG::dodgeModeSwitchThread;

void ChassisLG::init(tprio_t dodge_thread_prio_) {
    dodge_thread_prio = dodge_thread_prio_;
}

ChassisLG::action_t ChassisLG::get_action() {
    return action;
}

void ChassisLG::set_action(ChassisLG::action_t value) {
    action = value;
    if (action == STOP_MODE) {
        ChassisSKD::set_mode(ChassisSKD::STOP_MODE);
    } else if (action == FOLLOW_MODE) {
        ChassisSKD::set_mode(ChassisSKD::GIMBAL_COORDINATE_MODE);
        apply_target();
    } else if (action == DODGE_MODE) {
        ChassisSKD::set_mode(ChassisSKD::GIMBAL_COORDINATE_MODE);
        target_theta = DODGE_MODE_THETA;
        if (!dodgeModeSwitchThread.started) {
            dodgeModeSwitchThread.start(dodge_thread_prio);
        }
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
    started = true;
    setName("CH_Dodge");
    while(!shouldTerminate()) {

        if (action != DODGE_MODE) {
            started = false;
            exit(0);
            continue;
        }

        target_theta = -target_theta;
        apply_target();

        sleep(TIME_MS2I(DODGE_MODE_SWITCH_INTERVAL));
    }
}