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
//#include "chassis_scheduler.h"
//#include "remote_interpreter.h"

ChassisLG::action_t ChassisLG::action = FORCED_RELAX_MODE;
float ChassisLG::target_vx;
float ChassisLG::target_vy;
float ChassisLG::target_theta;

float ChassisLG::dodge_mode_theta_ = 0;
float ChassisLG::biased_angle_ = 0;
tprio_t ChassisLG::dodge_thread_prio;
ChassisLG::DodgeModeSwitchThread ChassisLG::dodgeModeSwitchThread;
chibios_rt::ThreadReference ChassisLG::dodgeThreadReference;


void ChassisLG::init(tprio_t dodge_thread_prio_, float dodge_mode_theta, float biased_angle) {
    dodge_thread_prio = dodge_thread_prio_;
    dodge_mode_theta_ = dodge_mode_theta;
    biased_angle_ = biased_angle;
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
//        ChassisSKD::load_pid_params(CHASSIS_FOLLOW_PID_THETA2V_PARAMS, CHASSIS_PID_V2I_PARAMS);
        ChassisSKD::set_mode(ChassisSKD::FORCED_RELAX_MODE);
    } else if (action == FOLLOW_MODE) {
        ChassisSKD::load_pid_params(CHASSIS_FOLLOW_PID_THETA2V_PARAMS, CHASSIS_PID_V2I_PARAMS);
        ChassisSKD::set_mode(ChassisSKD::GIMBAL_COORDINATE_MODE);
        apply_target();
    } else if (action == DODGE_MODE) {
        ChassisSKD::load_pid_params(CHASSIS_CLIP_PID_THETA2V_PARAMS, CHASSIS_CLIP_PID_V2I_PARAMS);
        ChassisSKD::set_mode(ChassisSKD::GIMBAL_COORDINATE_MODE);
        target_theta = dodge_mode_theta_;
        // Resume the thread
        chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
        if (!dodgeModeSwitchThread.started) {
            dodgeModeSwitchThread.started = true;
            chSchWakeupS(dodgeThreadReference.getInner(), 0);
        }
        chSysUnlock();  /// --- EXIT S-Locked state ---
    }
    Referee::set_client_light(USER_CLIENT_DODGE_MODE_LIGHT, (action == DODGE_MODE));
    // Sending client data will be complete by higher level thread
}

void ChassisLG::set_target(float vx, float vy) {
    target_vx = vx;
    target_vy = vy;
    if (action == FOLLOW_MODE) {
        target_theta = 0;
    }
#if defined(HERO)
    if (action == DODGE_MODE) {
        target_vx -= 500;
    }
#endif
    // For DODGE_MODE keep current target_theta unchanged
    apply_target();
}

void ChassisLG::apply_target() {
    ChassisSKD::set_target(target_vx, target_vy, target_theta);
}

float ChassisLG::get_actual_velocity(ChassisSKD::motor_id_t motor){
    return ChassisSKD::get_actual_velocity(motor);
}

void ChassisLG::DodgeModeSwitchThread::main() {

    setName("Chassis_Dodge");
    while(!shouldTerminate()) {

        chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
        if (action != DODGE_MODE) {
            started = false;
            chSchGoSleepS(CH_STATE_SUSPENDED);
        }
        chSysUnlock();  /// --- EXIT S-Locked state ---

        if (!ABS_IN_RANGE(ChassisSKD::get_actual_theta() - (-target_theta), 10)) {
            target_theta = -biased_angle_ - target_theta;
        }
        /**
         * If next target_theta is too close to current theta (may due to gimbal rotation), do not switch target to
         * create large difference to avoid pause
         */
        if (Remote::key.w || Remote::key.s || Remote::key.a || Remote::key.d ) {
            ChassisSKD::load_pid_params(CHASSIS_DODGE_PID_THETA2V_PARAMS, CHASSIS_PID_V2I_PARAMS);
        } else {
            ChassisSKD::load_pid_params(CHASSIS_CLIP_PID_THETA2V_PARAMS, CHASSIS_CLIP_PID_V2I_PARAMS);
        }
        apply_target();

        sleep(TIME_MS2I(DODGE_MODE_SWITCH_INTERVAL));
    }
}

/** @} */