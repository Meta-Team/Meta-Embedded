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


PIDController ChassisLG::dodge_omega_power_pid;
PIDController ChassisLG::auto_straightening_pid;

float ChassisLG::target_vx = 0.0f;
float ChassisLG::target_vy = 0.0f;
float ChassisLG::target_omega = 0.0f;
ChassisLG::chassis_mode_t ChassisLG::chassis_mode;
ChassisLG::MotionControllerThread ChassisLG::motion_controller_thread;
#ifdef ENABLE_REFEREE
ChassisLG::CapacitorSetThread ChassisLG::capacitor_set_thread;
#endif

void ChassisLG::set_target(float vx, float vy) {
    target_vx = vx;
    target_vy = vy;
    // For DODGE_MODE keep current target_theta unchanged
}

void ChassisLG::init(tprio_t dodge_thread_prio_, tprio_t cap_power_set_thread_prio_, tprio_t dodge_mode_max_omega) {
    motion_controller_thread.start(dodge_thread_prio_);
#if defined(ENABLE_SUBPITCH)
    capacitor_set_thread.start(cap_power_set_thread_prio_);
#endif
}

void ChassisLG::set_mode(ChassisLG::chassis_mode_t mode) {
    switch (mode) {
        case FORCE_RELAX_MODE:
            MecanumChassisSKD::set_mode(SKDBase::FORCED_RELAX_MODE);
            break;
        case CHASSIS_REF_MODE:
            MecanumChassisSKD::set_mode(SKDBase::CHASSIS_REF_MODE);
            target_omega = 0;
            break;
        case GIMBAL_REF_MODE:
            // Fall in.
        case DODGE:
            MecanumChassisSKD::set_mode(SKDBase::GIMBAL_REF_MODE);
            break;
    }
}

void ChassisLG::set_target_omega(float omega) {
    ChassisLG::target_omega = omega;
}

void ChassisLG::set_auto_straightening_pid_params(PIDControllerBase::pid_params_t params) {
    auto_straightening_pid.change_parameters(params);
}

void ChassisLG::set_dodge_omega_power_pid(PIDControllerBase::pid_params_t params) {
    dodge_omega_power_pid.change_parameters(params);
}

ChassisLG::chassis_mode_t ChassisLG::get_mode() {
    return chassis_mode;
}

void ChassisLG::MotionControllerThread::main() {
    setName("ChassisLG");
    while(!shouldTerminate()) {
        chSysLock();
        switch (chassis_mode) {
            case FORCE_RELAX_MODE:
                {
                    MecanumChassisSKD::set_mode(SKDBase::FORCED_RELAX_MODE);
                }
                break;
            case CHASSIS_REF_MODE:
                {
                    MecanumChassisSKD::set_target(target_vx, target_vy, target_omega);
                }

                break;
            case GIMBAL_REF_MODE:
                {
                    float yaw_accumulate_angle = CANMotorIF::motor_feedback[CANMotorCFG::YAW].accumulate_angle();
                    float yaw_auto_straightening_angle = (float)((int)(yaw_accumulate_angle/360.0f)) * 360.0f;
                    if (yaw_accumulate_angle - yaw_auto_straightening_angle > 180.0f) yaw_auto_straightening_angle += 360.0f;
                    target_omega = ChassisLG::auto_straightening_pid.calc(yaw_accumulate_angle, yaw_auto_straightening_angle);
                    MecanumChassisSKD::set_target(target_vx, target_vy, target_omega);
                }
                break;
            case DODGE:
                {
                    // Control the target omega to keep super capacitor voltage at 20V.
                    if (CapacitorIF::capacitor_voltage != 0) {
                        float voltage_decrement = 24 - CapacitorIF::capacitor_voltage;
                        target_omega = (dodge_omega_power_pid.calc(voltage_decrement, 4));
                        VAL_CROP(target_omega, 720.0f, 0.0f);
                    } else {
                        target_omega = 360;
                        ///target_omega = (dodge_omega_power_pid.calc((float)Referee::power_heat_data.chassis_power, (float)Referee::game_robot_state.chassis_power_limit));
                        VAL_CROP(target_omega, 720.0f, 0.0f);
                    }
                    VAL_CROP(target_omega, 720.0f, 0.0f);
                    MecanumChassisSKD::set_target(target_vx, target_vy, target_omega);
                }
                break;
        }
        chSysUnlock();
        sleep(TIME_MS2I(CHASSIS_LG_INTERVAL));
    }
}
#if defined(ENABLE_REFEREE)
void ChassisLG::CapacitorSetThread::main() {
    setName("CapacitorSetThd");
    while(!shouldTerminate()) {
        if ((float) Referee::power_heat.chassis_power_buffer - Referee::power_heat.chassis_power * 0.1 > 5.0) {
            CapacitorIF::set_power(95.0);
        } else {
            CapacitorIF::set_power((float) Referee::robot_state.chassis_power_limit * 0.9f);
        }
        sleep(TIME_MS2I(CAPACITOR_SET_INTERVAL));
    }
}
#endif
/** @} */