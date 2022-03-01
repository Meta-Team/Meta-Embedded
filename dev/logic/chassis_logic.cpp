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

float ChassisLG::target_vx = 0.0f;
float ChassisLG::target_vy = 0.0f;
float ChassisLG::target_omega = 0.0f;
ChassisLG::chassis_mode_t ChassisLG::chassis_mode;
ChassisLG::ChassisLGThread ChassisLG::chassis_logic_thread;

#if (FALSE)
ChassisLG::action_t ChassisLG::action = FORCED_RELAX_MODE;
float ChassisLG::target_theta;
float ChassisLG::dodge_mode_max_omega_ = 200.0f;
float ChassisLG::dodge_mode_min_omega_ = 200.0f;
int ChassisLG::dodge_mode_randomize_min_time_ = 2000.0f; // min rotate time for a constant speed[ms]
int ChassisLG::dodge_mode_randomize_max_time_ = 6000.0f; // max rotate time for a constant speed[ms]
float ChassisLG::biased_angle_ = 0;
tprio_t ChassisLG::dodge_thread_prio;
ChassisLG::DodgeModeSwitchThread ChassisLG::dodgeModeSwitchThread;
chibios_rt::ThreadReference ChassisLG::dodgeThreadReference;
ChassisLG::CapacitorPowerSetThread ChassisLG::capacitorPowerSetThread;
PIDController ChassisLG::dodge_omega_power_pid;

void ChassisLG::init(tprio_t dodge_thread_prio_, tprio_t cap_power_set_thread_prio_, float dodge_mode_max_omega,
                     float biased_angle, PIDController::pid_params_t omega_power_pid) {
    dodge_thread_prio = dodge_thread_prio_;
    dodge_mode_max_omega_ = dodge_mode_max_omega;
    biased_angle_ = biased_angle;
    dodgeModeSwitchThread.started = true;
    dodgeThreadReference = dodgeModeSwitchThread.start(dodge_thread_prio);
    capacitorPowerSetThread.start(cap_power_set_thread_prio_);
    dodge_omega_power_pid.change_parameters(omega_power_pid);
}

ChassisLG::action_t ChassisLG::get_action() {
    return action;
}

#endif

void ChassisLG::set_target(float vx, float vy) {
    target_vx = vx;
    target_vy = vy;
    // For DODGE_MODE keep current target_theta unchanged
}

void ChassisLG::init(tprio_t dodge_thread_prio_, tprio_t cap_power_set_thread_prio_, tprio_t dodge_mode_max_omega) {
    chassis_logic_thread.start(dodge_thread_prio_);
}

void ChassisLG::set_mode(ChassisLG::chassis_mode_t mode) {
    chSysLock();
    {
        ChassisLG::chassis_mode = mode;
        if(mode != FORCE_RELAX) {
            CANMotorCFG::enable_v2i[CANMotorCFG::FL] = true;
            CANMotorCFG::enable_v2i[CANMotorCFG::FR] = true;
            CANMotorCFG::enable_v2i[CANMotorCFG::BR] = true;
            CANMotorCFG::enable_v2i[CANMotorCFG::BL] = true;
        }
    }
    chSysUnlock();
}

void ChassisLG::set_target_omega(float omega) {
    chSysLock();
    {
        ChassisLG::target_omega = omega;
    }
    chSysUnlock();
}


// TODO: Re-enable the functions when ready
#if (FALSE)
void ChassisLG::apply_target() {
    if (action == FOLLOW_MODE) {
        ChassisSKD::set_target(target_vx, target_vy, target_theta);
    } else if (action == DODGE_MODE) {
        ChassisSKD::set_dodge_target(target_vx, target_vy, target_omega);
    }
}

void ChassisLG::DodgeModeSwitchThread::main() {

    setName("ChassisLG_Dodge");
    while (!shouldTerminate()) {

        chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
        {
            if (action != DODGE_MODE) {
                started = false;
                chSchGoSleepS(CH_STATE_SUSPENDED);
            }
        }
        chSysUnlock();  /// --- EXIT S-Locked state ---

        if (CapacitorIF::feedback->capacitor_voltage != 0) {
            float voltage_decrement = 20 - CapacitorIF::feedback->capacitor_voltage;
            target_omega = (dodge_omega_power_pid.calc(voltage_decrement, 2));
            VAL_CROP(target_omega, 720.0f, 0.0f);
        } else {
            target_omega = 360;
            ///target_omega = (dodge_omega_power_pid.calc((float)Referee::power_heat_data.chassis_power, (float)Referee::game_robot_state.chassis_power_limit));
            VAL_CROP(target_omega, 720.0f, 0.0f);
        }

        /**
         * If next target_theta is too close to current theta (may due to gimbal rotation), do not switch target to
         * create large difference to avoid pause
         */
        if (Remote::key.w || Remote::key.s || Remote::key.a || Remote::key.d) {
            ChassisSKD::load_pid_params(CHASSIS_DODGE_PID_THETA2V_PARAMS, CHASSIS_PID_V2I_PARAMS);
        } else {
            ChassisSKD::load_pid_params(CHASSIS_CLIP_PID_THETA2V_PARAMS, CHASSIS_CLIP_PID_V2I_PARAMS);
        }
        apply_target();

        sleep(TIME_MS2I(DODGE_MODE_SWITCH_INTERVAL));
    }
}

void ChassisLG::CapacitorPowerSetThread::main() {
    setName("Cap_Setting");
    while (!shouldTerminate()) {

        if ((float) Referee::power_heat.chassis_power_buffer - Referee::power_heat.chassis_power * 0.1 > 5.0) {
            CapacitorIF::set_power(95.0);
        } else {
            CapacitorIF::set_power((float) Referee::robot_state.chassis_power_limit * 0.9f);
        }

        sleep(TIME_MS2I(CAP_POWER_SET_INTERVAL));
    }
}
#endif

void ChassisLG::ChassisLGThread::main() {
    setName("ChassisLG");
    while(!shouldTerminate()) {
        switch (chassis_mode) {
            case ABS:
                chSysLock();
                {
                    MecanumChassisSKD::set_target(target_vx, target_vy, target_omega);
                }
                chSysUnlock();
                break;
            case FOLLOW:
                chSysLock();
                {

                }
                chSysUnlock();
                break;
            case DODGE:
                chSysLock();
                {

                }
                chSysUnlock();
                break;
            case FORCE_RELAX:
                chSysLock();
                {
                    CANMotorCFG::enable_v2i[CANMotorCFG::FL] = false;
                    CANMotorSKD::set_target_current(CANMotorCFG::FL, 0);
                    CANMotorCFG::enable_v2i[CANMotorCFG::FR] = false;
                    CANMotorSKD::set_target_current(CANMotorCFG::FR, 0);
                    CANMotorCFG::enable_v2i[CANMotorCFG::BR] = false;
                    CANMotorSKD::set_target_current(CANMotorCFG::BR, 0);
                    CANMotorCFG::enable_v2i[CANMotorCFG::BL] = false;
                    CANMotorSKD::set_target_current(CANMotorCFG::BL, 0);
                }
                chSysUnlock();
                break;
        }
        sleep(TIME_MS2I(VEL_DECOMPOSE_INTERVAL));
    }
}

/** @} */