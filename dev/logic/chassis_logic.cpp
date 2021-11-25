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
ChassisLG::VelocityDecomposeThread ChassisLG::velocityDecomposeThread;
ChassisLG::install_mode_t ChassisLG::install_mode = POSITIVE;
float ChassisLG::w_to_v_ratio = 0.0f;
float ChassisLG::v_to_wheel_angular_velocity = 0.0f;

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

void ChassisLG::set_action(ChassisLG::action_t value) {
    if (value == action) return;  // avoid repeating setting target_theta, etc.
    action = value;
    if (action == FORCED_RELAX_MODE) {

    } else if (action == FOLLOW_MODE) {

    } else if (action == DODGE_MODE) {
        chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
        if (!dodgeModeSwitchThread.started) {
            dodgeModeSwitchThread.started = true;
            chSchWakeupS(dodgeThreadReference.getInner(), 0);
        }
        chSysUnlock();  /// --- EXIT S-Locked state ---
    }
    // Sending client data will be complete by higher level thread
}
#endif

void ChassisLG::set_target(float vx, float vy, float omega) {
    target_vx = vx;
    target_vy = vy;
    target_omega = omega;
    // For DODGE_MODE keep current target_theta unchanged
}

void ChassisLG::init(tprio_t velocity_decompose, float wheel_base, float wheel_thread, float wheel_circumference) {
    velocityDecomposeThread.start(velocity_decompose);
    w_to_v_ratio = (wheel_base + wheel_thread) / 2.0f / 360.0f * 3.14159f;
    v_to_wheel_angular_velocity = (360.0f / wheel_circumference);
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

        if (SuperCapacitor::feedback->capacitor_voltage != 0) {
            float voltage_decrement = 20 - SuperCapacitor::feedback->capacitor_voltage;
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
            SuperCapacitor::set_power(95.0);
        } else {
            SuperCapacitor::set_power((float) Referee::robot_state.chassis_power_limit * 0.9f);
        }

        sleep(TIME_MS2I(CAP_POWER_SET_INTERVAL));
    }
}
#endif

/** @} */
void ChassisLG::VelocityDecomposeThread::main() {
    setName("VelocityKinematic");
    while(!shouldTerminate()) {
        CANMotorSKD::set_target_vel(CANMotorCFG::FR, (float)install_mode *
                                                     (target_vx-target_vy + target_omega * w_to_v_ratio) * v_to_wheel_angular_velocity);
        CANMotorSKD::set_target_vel(CANMotorCFG::FL, (float)install_mode *
                                                     (target_vx+target_vy + target_omega * w_to_v_ratio) * v_to_wheel_angular_velocity);
        CANMotorSKD::set_target_vel(CANMotorCFG::BL, (float)install_mode *
                                                     (-target_vx+target_vy+ target_omega * w_to_v_ratio) * v_to_wheel_angular_velocity);
        CANMotorSKD::set_target_vel(CANMotorCFG::BR, (float)install_mode *
                                                     (-target_vx-target_vy+ target_omega * w_to_v_ratio) * v_to_wheel_angular_velocity);
        sleep(TIME_MS2I(VEL_DECOMPOSE_INTERVAL));
    }
}
