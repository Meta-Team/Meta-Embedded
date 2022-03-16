//
// Created by Administrator on 2019/1/11 0011.
//

/**
 * @file    chassis_scheduler.cpp
 * @brief   Scheduler to control chassis to meet the target, including a thread to invoke PID calculation in period.
 *
 * @addtogroup chassis
 * @{
 */

#include "mecanum_chassis_scheduler.h"

float MecanumChassisSKD::target_vx = 0.0f;
float MecanumChassisSKD::target_vy = 0.0f;
float MecanumChassisSKD::target_omega = 0.0f;

MecanumChassisSKD::install_mode_t MecanumChassisSKD::install_mode = POSITIVE;
float MecanumChassisSKD::w_to_v_ratio = 0.0f;
float MecanumChassisSKD::v_to_wheel_angular_velocity = 0.0f;
float MecanumChassisSKD::chassis_gimbal_offset_ = 0.0f;

MecanumChassisSKD::SKDThread MecanumChassisSKD::skd_thread;
SKDBase::mode_t MecanumChassisSKD::mode;

void MecanumChassisSKD::init(tprio_t skd_prio, float wheel_base, float wheel_thread, float wheel_circumference, float gimbal_offset) {
    skd_thread.start(skd_prio);
    w_to_v_ratio = (wheel_base + wheel_thread) / 2.0f / 360.0f * 3.14159f;
    v_to_wheel_angular_velocity = (360.0f / wheel_circumference);
    chassis_gimbal_offset_ = gimbal_offset;
}

void MecanumChassisSKD::set_target(float vx, float vy, float omega) {
    target_vx = vx;
    target_vy = vy;
    target_omega = omega;
    // For DODGE_MODE keep current target_theta unchanged
}

void MecanumChassisSKD::set_mode(SKDBase::mode_t mode_) {
    MecanumChassisSKD::mode = mode;
    if(mode != FORCED_RELAX_MODE) {
        CANMotorCFG::enable_v2i[CANMotorCFG::FL] = true;
        CANMotorCFG::enable_v2i[CANMotorCFG::FR] = true;
        CANMotorCFG::enable_v2i[CANMotorCFG::BR] = true;
        CANMotorCFG::enable_v2i[CANMotorCFG::BL] = true;
    } else {
        CANMotorCFG::enable_v2i[CANMotorCFG::FL] = false;
        CANMotorCFG::enable_v2i[CANMotorCFG::FR] = false;
        CANMotorCFG::enable_v2i[CANMotorCFG::BR] = false;
        CANMotorCFG::enable_v2i[CANMotorCFG::BL] = false;
        CANMotorSKD::set_target_current(CANMotorCFG::FL, 0);
        CANMotorSKD::set_target_current(CANMotorCFG::FR, 0);
        CANMotorSKD::set_target_current(CANMotorCFG::BR, 0);
        CANMotorSKD::set_target_current(CANMotorCFG::BL, 0);
    }
}

void MecanumChassisSKD::SKDThread::main() {
    setName("ChassisSKDThread");
    while(!shouldTerminate()) {
        chSysLock();  /// --- ENTER S-Locked state. DO NOT use LOG, printf, non S/I-Class functions or return ---
        {
            switch (mode) {

                case FORCED_RELAX_MODE:
                    CANMotorCFG::enable_v2i[CANMotorCFG::FL] = false;
                    CANMotorCFG::enable_v2i[CANMotorCFG::FR] = false;
                    CANMotorCFG::enable_v2i[CANMotorCFG::BR] = false;
                    CANMotorCFG::enable_v2i[CANMotorCFG::BL] = false;
                    CANMotorSKD::set_target_current(CANMotorCFG::FL, 0);
                    CANMotorSKD::set_target_current(CANMotorCFG::FR, 0);
                    CANMotorSKD::set_target_current(CANMotorCFG::BR, 0);
                    CANMotorSKD::set_target_current(CANMotorCFG::BL, 0);
                    break;
                case CHASSIS_REF_MODE:
                    CANMotorSKD::set_target_vel(CANMotorCFG::FR, (float)install_mode *
                                                                 (target_vx-target_vy + target_omega * w_to_v_ratio) * v_to_wheel_angular_velocity);
                    CANMotorSKD::set_target_vel(CANMotorCFG::FL, (float)install_mode *
                                                                 (target_vx+target_vy + target_omega * w_to_v_ratio) * v_to_wheel_angular_velocity);
                    CANMotorSKD::set_target_vel(CANMotorCFG::BL, (float)install_mode *
                                                                 (-target_vx+target_vy+ target_omega * w_to_v_ratio) * v_to_wheel_angular_velocity);
                    CANMotorSKD::set_target_vel(CANMotorCFG::BR, (float)install_mode *
                                                                 (-target_vx-target_vy+ target_omega * w_to_v_ratio) * v_to_wheel_angular_velocity);
                case GIMBAL_REF_MODE:

                    float gimbal_offset_angle = CANMotorIF::motor_feedback[CANMotorCFG::YAW].actual_angle;
                    float chassis_vx = target_vx * cosf(gimbal_offset_angle / 180.0f * PI)
                                       - target_vy * sinf(gimbal_offset_angle / 180.0f * PI)
                                       - target_omega / 180.0f * PI * chassis_gimbal_offset_;
                    float chassis_vy = target_vx * sinf(gimbal_offset_angle / 180.0f * PI)
                                       + target_vy * cosf(gimbal_offset_angle / 180.0f * PI);

                    CANMotorSKD::set_target_vel(CANMotorCFG::FR, (float)install_mode *
                                                                 (chassis_vx-chassis_vy + target_omega * w_to_v_ratio) * v_to_wheel_angular_velocity);
                    CANMotorSKD::set_target_vel(CANMotorCFG::FL, (float)install_mode *
                                                                 (chassis_vx+chassis_vy + target_omega * w_to_v_ratio) * v_to_wheel_angular_velocity);
                    CANMotorSKD::set_target_vel(CANMotorCFG::BL, (float)install_mode *
                                                                 (-chassis_vx+chassis_vy+ target_omega * w_to_v_ratio) * v_to_wheel_angular_velocity);
                    CANMotorSKD::set_target_vel(CANMotorCFG::BR, (float)install_mode *
                                                                 (-chassis_vx-chassis_vy+ target_omega * w_to_v_ratio) * v_to_wheel_angular_velocity);
                    break;
            }
        }
        chSysUnlock();  /// --- EXIT S-Locked state ---
        sleep(TIME_MS2I(SKD_INTERVAL));
    }

}
/// TODO: Re-Enable These functions.
#ifdef ENABLE_CHASSIS_SHELL
const Shell::Command ChassisSKD::shellCommands[] = {
        {"_c",              nullptr,                                        ChassisSKD::cmdInfo,           nullptr},
        {"_c_enable_fb",    "Channel/All Feedback{Disabled,Enabled}",       ChassisSKD::cmdEnableFeedback, nullptr},
        {"_c_pid",          "Channel [kp] [ki] [kd] [i_limit] [out_limit]", ChassisSKD::cmdPID,            nullptr},
        {"_c_enable_motor", "All Motor{Disabled,Enabled}",                  ChassisSKD::cmdEnableMotor,    nullptr},
        {nullptr,           nullptr,                                        nullptr,                       nullptr}
};

DEF_SHELL_CMD_START(ChassisSKD::cmdInfo)
    Shell::printf("_c:Chassis" ENDL);
    Shell::printf("_c/Front_Right:Velocity{Actual,Target} Current{Actual,Target}" ENDL);
    Shell::printf("_c/Front_Left:Velocity{Actual,Target} Current{Actual,Target}" ENDL);
    Shell::printf("_c/Back_Left:Velocity{Actual,Target} Current{Actual,Target}" ENDL);
    Shell::printf("_c/Back_Right:Velocity{Actual,Target} Current{Actual,Target}" ENDL);
    Shell::printf("_c/Theta:Angle{Actual,Target}" ENDL);
    return true;
DEF_SHELL_CMD_END

static bool feedbackEnabled[ChassisSKD::MOTOR_COUNT + 1] = {false, false, false, false, false};

void ChassisSKD::cmdFeedback(void *) {
    for (int i = 0; i <= MOTOR_COUNT; i++) {
        if (feedbackEnabled[i]) {
            Shell::printf("_c%d %.2f %.2f %d %d" ENDL, i,
                          ChassisIF::feedback[i]->actual_velocity, target_velocity[i],
                          ChassisIF::feedback[i]->actual_current, *ChassisIF::target_current[i]);
        }
    }
    if (feedbackEnabled[4]) {
        Shell::printf("_c4 %.2f %.2f" ENDL, actual_theta, target_theta);
    }
}

DEF_SHELL_CMD_START(ChassisSKD::cmdEnableFeedback)
    int id;
    bool enabled;
    if (!Shell::parseIDAndBool(argc, argv, 5, id, enabled)) return false;
    if (id == -1) {
        for (bool &e : feedbackEnabled) e = enabled;
    } else {
        feedbackEnabled[id] = enabled;
    }
    return true;
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(ChassisSKD::cmdEnableMotor)
    int id;
    bool enabled;
    if (!Shell::parseIDAndBool(argc, argv, 0, id, enabled)) return false;
    if (id == -1) {
        motor_enabled = enabled;
        return true;
    } else {
        return false;
    }
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(ChassisSKD::cmdPID)
    if (argc < 1) return false;

    unsigned id = Shell::atoi(argv[0]);
    if (id >= MOTOR_COUNT + 1) return false;

    PIDController *pid = id == MOTOR_COUNT + 1 ? &a2v_pid : &v2i_pid[id];
    if (argc == 1) {
        pid_params_t params = pid->get_parameters();
        Shell::printf("_c_pid %u %.2f %.2f %.2f %.2f %.2f" SHELL_NEWLINE_STR,
                      id, params.kp, params.ki, params.kd, params.i_limit, params.out_limit);
    } else if (argc == 7) {
        pid->change_parameters({Shell::atof(argv[2]),
                                Shell::atof(argv[3]),
                                Shell::atof(argv[4]),
                                Shell::atof(argv[5]),
                                Shell::atof(argv[6])});
    } else {
        return false;
    }

    return true;
DEF_SHELL_CMD_END
#endif

/** @} */