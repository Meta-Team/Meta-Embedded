//
// Created by kerui on 2021/7/14.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "shell.h"

#include "thread_priorities.h"

#include "can_interface.h"
#include "can_motor_controller.h"

#include "ahrs.h"
#include "remote_interpreter.h"

#include "buzzer_scheduler.h"

using namespace chibios_rt;

const char *motor_name[10] = {
        "Front Left",
        "Front Right",
        "Back Right",
        "Back Left",
        "Yaw",
        "PITCH",
        "SUB_PITCH",
        "BULLET_LOADER",
        "FW_UP",
        "FW_DOWN"};

/// Define constants
constexpr unsigned USER_THREAD_INTERVAL = 7;  // [ms]

float gimbal_yaw_target_angle_ = 0;
float gimbal_rc_yaw_max_speed = 180;  // [degree/s]

float gimbal_pitch_min_angle = -30; // down range for pitch [degree]
float gimbal_pitch_max_angle = 10; //  up range for pitch [degree]

bool is_shooting = false;
float loader_velocity = 20.0f;
float friction_velocity = 100.0f;

/// AHRS PARAMETERS
/// Depends on the installation direction of the board
#define ON_BOARD_AHRS_MATRIX {{0.0f, -1.0f, 0.0f}, \
                              {1.0f, 0.0f, 0.0f}, \
                              {0.0f, 0.0f, 1.0f}}

#define GIMBAL_ANGLE_INSTALLATION_MATRIX {{1.0f, 0.0f, 0.0f}, \
                                          {0.0f, 1.0f, 0.0f}, \
                                          {0.0f, 0.0f, -1.0f}}


#define GIMBAL_GYRO_INSTALLATION_MATRIX {{0.0f,  -1.0f, 0.0f}, \
                                         {0.0f,  0.0f,  1.0f}, \
                                         {1.0f, 0.0f,  0.0f}}

static const Matrix33 ON_BOARD_AHRS_MATRIX_ = ON_BOARD_AHRS_MATRIX;
static const Matrix33 GIMBAL_ANGLE_INSTALLATION_MATRIX_ = GIMBAL_ANGLE_INSTALLATION_MATRIX;
static const Matrix33 GIMBAL_GYRO_INSTALLATION_MATRIX_ = GIMBAL_GYRO_INSTALLATION_MATRIX;
#define MPU6500_STORED_GYRO_BIAS {0.491007685, -1.024637818, -0.371067702}

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);
AHRSOnBoard ahrs;

DEF_SHELL_CMD_START(cmd_motor_angle_enable)
    (void) argv;
    if(argc != 1){
        return false;
    }
    unsigned motor_id = Shell::atoi(argv[0]);
    if (motor_id >= CANMotorCFG::MOTOR_COUNT) {
        Shell::printf("Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return false;
    }
    CANMotorCFG::enable_a2v[motor_id] = true;
    CANMotorCFG::enable_v2i[motor_id] = true;
    return true; // command executed successfully
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(cmd_motor_angle_disable)
    (void) argv;
    if(argc != 1){
        return false;
    }
    unsigned motor_id = Shell::atoi(argv[0]);
    if (motor_id >= CANMotorCFG::MOTOR_COUNT) {
        Shell::printf("Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return false;
    }
    CANMotorCFG::enable_a2v[motor_id] = false;
    CANMotorCFG::enable_v2i[motor_id] = false;
    CANMotorController::set_target_current((CANMotorCFG::motor_id_t)motor_id, 0);
    return true; // command executed successfully
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(cmd_motor_velocity_enable)
    (void) argv;
    if(argc != 1){
        return false;
    }
    unsigned motor_id = Shell::atoi(argv[0]);
    if (motor_id >= CANMotorCFG::MOTOR_COUNT) {
        Shell::printf("Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return true;
    }
    CANMotorCFG::enable_a2v[motor_id] = false;
    CANMotorCFG::enable_v2i[motor_id] = true;
    return true; // command executed successfully
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(cmd_motor_velocity_disable)
    (void) argv;
    if(argc != 1){
        return false;
    }
    unsigned motor_id = Shell::atoi(argv[0]);
    CANMotorCFG::enable_a2v[motor_id] = false;
    CANMotorCFG::enable_v2i[motor_id] = false;
    CANMotorController::set_target_current((CANMotorCFG::motor_id_t)motor_id, 0);
    return true; // command executed successfully
DEF_SHELL_CMD_END


DEF_SHELL_CMD_START(cmd_get_sid)
    (void) argv;
    unsigned motor_id = Shell::atoi(argv[0]);
    if (motor_id >= CANMotorCFG::MOTOR_COUNT) {
        Shell::printf("Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return false;
    }
    Shell::printf("motor type: %d, motor sid: %x" SHELL_NEWLINE_STR,
                  CANMotorCFG::CANMotorProfile[motor_id].motor_type,
                  CANMotorCFG::CANMotorProfile[motor_id].CAN_SID);
    return true; // command executed successfully
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(cmd_enable_feedback)
    if(argc != 1){
        return false;
    }
    unsigned feedback_motor_id = Shell::atoi(argv[0]);
    if (feedback_motor_id >= CANMotorCFG::MOTOR_COUNT) {
        Shell::printf("Invalid feedback ID %d" SHELL_NEWLINE_STR, feedback_motor_id);
        return false;
    }
    Shell::printf("%s feedback enabled" SHELL_NEWLINE_STR, motor_name[feedback_motor_id]);
    CANMotorController::shell_display((CANMotorCFG::motor_id_t) feedback_motor_id, true);
    return true; // command executed successfully
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(cmd_disable_feedback)
    (void) argv;
    if (argc != 1) {
        return false;
    }
    unsigned feedback_motor_id = Shell::atoi(argv[0]);
    if (feedback_motor_id >= CANMotorCFG::MOTOR_COUNT) {
        Shell::printf("Invalid feedback ID %d" SHELL_NEWLINE_STR, feedback_motor_id);
        return false;
    }
    Shell::printf("%s feedback disabled" SHELL_NEWLINE_STR, motor_name[feedback_motor_id]);
    CANMotorController::shell_display((CANMotorCFG::motor_id_t) feedback_motor_id, false);
    return true; // command executed successfully
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(cmd_set_param)
    (void) argv;
    unsigned motor_id = Shell::atoi(argv[0]);
    if (motor_id >= CANMotorCFG::MOTOR_COUNT) {
        Shell::printf("Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return false;
    }
    unsigned pid_id = Shell::atoi(argv[1]);
    if (pid_id > 1) {
        Shell::printf("Invalid pid ID %d" SHELL_NEWLINE_STR, pid_id);
        return false;
    }
    PIDController::pid_params_t pid_param = {Shell::atof(argv[2]),
                                             Shell::atof(argv[3]),
                                             Shell::atof(argv[4]),
                                             Shell::atof(argv[5]),
                                             Shell::atof(argv[6])};

    CANMotorController::load_PID_params((CANMotorCFG::motor_id_t) motor_id,  pid_id == 0, pid_param);

    Shell::printf("ps!" SHELL_NEWLINE_STR);
    return true; // command executed successfully
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(cmd_echo_param)
    (void) argv;
    unsigned motor_id = Shell::atoi(argv[0]);
    if (motor_id >= CANMotorCFG::MOTOR_COUNT) {
        Shell::printf("Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return false;
    }

    unsigned pid_id = Shell::atoi(argv[1]);
    if (pid_id > 1) {
        // TODO, pid_id=2 for autostraightening
        Shell::printf("Invalid pid ID %d" SHELL_NEWLINE_STR, pid_id);
        return false;
    }

    PIDController::pid_params_t pid_param = {0,0,0,0,0};
    pid_param = CANMotorController::getPIDParams((CANMotorCFG::motor_id_t)motor_id, (pid_id == 0));
    Shell::printf("!kp,%u,%u,%.2f,%.2f,%.2f,%.2f,%.2f" SHELL_NEWLINE_STR,motor_id,pid_id,
                 pid_param.kp, pid_param.ki, pid_param.kd, pid_param.i_limit, pid_param.out_limit);
    return true; // command executed successfully
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(cmd_set_target_angle)
    (void) argv;
    unsigned motor_id = Shell::atoi(argv[0]);
    if (motor_id >= CANMotorCFG::MOTOR_COUNT) {
        Shell::printf("Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return false;
    }
    float angle = Shell::atof(argv[1]);
    CANMotorController::set_target_angle((CANMotorCFG::motor_id_t)motor_id, angle);
    Shell::printf("ps!" SHELL_NEWLINE_STR);
    return true; // command executed successfully
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(cmd_set_target_velocity)
    (void) argv;
    unsigned motor_id = Shell::atoi(argv[0]);
    if (motor_id >= CANMotorCFG::MOTOR_COUNT) {
        Shell::printf("Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return false;
    }
    float velocity = Shell::atof(argv[1]);
    CANMotorController::set_target_vel((CANMotorCFG::motor_id_t)motor_id, velocity);
    Shell::printf("ps!" SHELL_NEWLINE_STR);
    return true; // command executed successfully
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(cmd_echo_actual_angle)
    (void) argv;
    unsigned motor_id = Shell::atoi(argv[0]);
    if (motor_id >= CANMotorCFG::MOTOR_COUNT) {
        Shell::printf("Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return false;
    }
    float angle = 0;
    angle = CANMotorIF::motor_feedback[motor_id].accumulate_angle();
    Shell::printf("%.2f" SHELL_NEWLINE_STR, angle);
    return true; // command executed successfully
DEF_SHELL_CMD_END

DEF_SHELL_CMD_START(cmd_echo_raw_angle)
    (void) argv;
    unsigned motor_id = Shell::atoi(argv[0]);
    if (motor_id >= CANMotorCFG::MOTOR_COUNT) {
        Shell::printf("Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return false;
    }
    Shell::printf("%d" SHELL_NEWLINE_STR, CANMotorIF::motor_feedback[motor_id].rotor_angle_raw);
    return true; // command executed successfully
DEF_SHELL_CMD_END


class TopControlThread : public BaseStaticThread<512> {
private:
    void main() final {
        setName("TopControlThread");
        Remote::rc_status_t previous_rcs1_state = Remote::rc.s1;
        CANMotorCFG::motor_id_t vel_group[] = {CANMotorCFG::YAW, CANMotorCFG::PITCH};
        CANMotorCFG::motor_id_t angle_group[] = {CANMotorCFG::FL, CANMotorCFG::FR,
                                                 CANMotorCFG::BL, CANMotorCFG::BR,
                                                 CANMotorCFG::FW_UP, CANMotorCFG::FW_DOWN,
                                                 CANMotorCFG::BULLET_LOADER};
        while (!shouldTerminate()) {

            if (Remote::rc.s1 == Remote::S_DOWN) {


            }

            if(Remote::rc.s1 == Remote::S_UP){
                /// Safe Mode
                for (int i = 0; i < CANMotorCFG::MOTOR_COUNT; i++) {
                    CANMotorCFG::enable_a2v[i] = false;
                    CANMotorCFG::enable_v2i[i] = false;
                    CANMotorController::set_target_current((CANMotorCFG::motor_id_t)i, 0);
                }
            }

            previous_rcs1_state = Remote::rc.s1;

            /// Final, since user thread is not time sensitive, we use fixed time sleep interval. (No need to adjust sleep time for constant interval)
            sleep(TIME_MS2I(USER_THREAD_INTERVAL));
        }
    }
} topControlThread;

// Command lists for gimbal controller test and adjustments
Shell::Command mainProgramCommands[] = {
        {"set_enable_a",      "motor_id",                                                             cmd_motor_angle_enable,    nullptr},
        {"set_disable_a",     "motor_id",                                                             cmd_motor_angle_disable,   nullptr},
        {"set_enable_v",      "motor_id",                                                             cmd_motor_velocity_enable, nullptr},
        {"set_disable_v",     "motor_id",                                                             cmd_motor_velocity_disable,nullptr},
        {"get_sid",           "motor_id",                                                             cmd_get_sid,               nullptr},
        {"fb_enable",         "motor_id",                                                             cmd_enable_feedback,       nullptr},
        {"fb_disable",        "motor_id",                                                             cmd_disable_feedback,      nullptr},
        {"set_pid",           "motor_id pid_id(0: angle_to_v, 1: v_to_i) kp ki kd i_limit out_limit", cmd_set_param,             nullptr},
        {"echo_pid",          "motor_id pid_id(0: angle_to_v, 1: v_to_i)",                            cmd_echo_param,            nullptr},
        {"set_target_angle",  "motor_id target_angle",                                                cmd_set_target_angle,      nullptr},
        {"set_target_vel",    "motor_id target_vel",                                                  cmd_set_target_velocity,  nullptr},
        {"echo_actual_angle", "motor_id",                                                             cmd_echo_actual_angle,    nullptr},
        {"echo_raw_angle",    "motor_id",                                                             cmd_echo_raw_angle,       nullptr},
        {nullptr, nullptr, nullptr, nullptr}
};




int main() {
    /*** --------------------------- Period 0. Fundamental Setup --------------------------- ***/

    halInit();
    chibios_rt::System::init();

    // Enable power of bullet loader motor
    palSetPadMode(GPIOH, GPIOH_POWER1_CTRL, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(GPIOH, GPIOH_POWER1_CTRL);

    /*** ---------------------- Period 1. Modules Setup and Self-Check ---------------------- ***/

    /// Preparation of Period 1
    LED::all_off();

    /// Setup Shell
    Shell::start(THREAD_SHELL_PRIO);
    Shell::addCommands(mainProgramCommands);
    chThdSleepMilliseconds(50);  // wait for logo to print :)

    BuzzerSKD::init(THREAD_BUZZER_SKD_PRIO);

    /// Setup CAN1 & CAN2
    can1.start(THREAD_CAN1_RX_PRIO);
    can2.start(THREAD_CAN2_RX_PRIO);
    chThdSleepMilliseconds(5);

    /// Setup On-Board AHRS
    ahrs.load_calibration_data(MPU6500_STORED_GYRO_BIAS);
    ahrs.start(ON_BOARD_AHRS_MATRIX_, THREAD_AHRS_PRIO);
    chThdSleepMilliseconds(5);

    /// Setup Remote
    Remote::start();


    CANMotorController::start(THREAD_MOTOR_SKD_PRIO, THREAD_FEEDBACK_SKD_PRIO, &can1, &can2);
    chThdSleepMilliseconds(2000);  // wait for C610 to be online and friction wheel to reset

    /// Setup Red Spot Laser
    palSetPad(GPIOG, GPIOG_RED_SPOT_LASER);  // enable the red spot laser


    /*** ------------ Period 2. Calibration and Start Logic Control Thread ----------- ***/

    /// Echo Gimbal Raws and Converted Angles
    LOG("Gimbal Yaw: %u, %f, Pitch: %u, %f" SHELL_NEWLINE_STR,
        CANMotorIF::motor_feedback[CANMotorCFG::YAW].rotor_angle_raw, CANMotorIF::motor_feedback[CANMotorCFG::YAW].accumulate_angle(),
        CANMotorIF::motor_feedback[CANMotorCFG::PITCH].rotor_angle_raw, CANMotorIF::motor_feedback[CANMotorCFG::PITCH].accumulate_angle());

    /// Start SKDs
    topControlThread.start(THREAD_USER_PRIO);

    /// Complete Period 2
    BuzzerSKD::play_sound(BuzzerSKD::sound_startup_intel);  // Now play the startup sound


    /*** ------------------------ Period 3. End of main thread ----------------------- ***/

    // Entering empty loop with low priority
#if CH_CFG_NO_IDLE_THREAD  // See chconf.h for what this #define means.
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When vehicle() quits, the vehicle thread will somehow enter an infinite loop, so we set the
    // priority to lowest before quitting, to let other threads run normally
    chibios_rt::BaseThread::setPriority(IDLEPRIO);
#endif
    return 0;
}