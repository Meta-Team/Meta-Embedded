//
// Created by kerui on 2021/7/14.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "debug/shell/shell.h"

#include "can_interface.h"
#include "ahrs.h"
#include "sd_card_interface.h"
#include "remote_interpreter.h"

#include "buzzer_scheduler.h"

#include "scheduler/gimbal_scheduler.h"
#include "scheduler/shoot_scheduler.h"

#include "vehicle/hero/vehicle_hero.h"

using namespace chibios_rt;

const char *motor_name[10] = {
        "yaw",
        "pitch",
        "sub_pitch",
        "bullet",
        "fw_left",
        "fw_right",
        "ahrs",
        "gyro",
        "accel",
        "magnet"};

// Calculation interval for gimbal thread
unsigned const GIMBAL_THREAD_INTERVAL = 1;    // [ms]
unsigned const GIMBAL_FEEDBACK_INTERVAL = 25; // [ms]

int const MAX_CURRENT = 30000;  // [mA]

bool feedback_enable[13];

// AHRS PARAMETERS
// Depends on the install direction of the board
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

#define THREAD_CAN1_PRIO                    (HIGHPRIO - 1)
#define THREAD_FEEDBACK_PRIO                (LOWPRIO + 6)

static GimbalIF::motor_can_config_t GIMBAL_MOTOR_CONFIG_[GimbalIF::MOTOR_COUNT] = GIMBAL_MOTOR_CONFIG;

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);
AHRSOnBoard ahrs;


/**
 * @brief enabled motors
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_set_enable(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "set_enable motor_id(0-5)");
        return;
    }
    unsigned motor_id = Shell::atoi(argv[0]);
    if (motor_id > 5) {
        chprintf(chp, "Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return;
    }
    if (motor_id < 3) {
        GimbalSKD::enable_motor((GimbalSKD::motor_id_t) motor_id);
    } else {
        ShootSKD::enable_motor(motor_id - 3);
    }
}

/**
 * @brief disabled motors
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_set_disable(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "set_disable motor_id(0-5)");
        return;
    }
    unsigned motor_id = Shell::atoi(argv[0]);
    if (motor_id > 5) {
        chprintf(chp, "Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return;
    }
    if (motor_id < 3) {
        GimbalSKD::disable_motor((GimbalSKD::motor_id_t) motor_id);
    } else {
        ShootSKD::disable_motor(motor_id - 3);
    }
}

static void cmd_get_sid(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "get_sid motor_id(0-5)");
        return;
    }
    unsigned motor_id = Shell::atoi(argv[0]);
    if (motor_id > 5) {
        chprintf(chp, "Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return;
    }
    chprintf(chp, "motor type: %d, motor sid: %d" SHELL_NEWLINE_STR, GimbalIF::feedback[(GimbalIF::motor_id_t) motor_id]->type, GimbalIF::feedback[(GimbalIF::motor_id_t) motor_id]->sid);
}

void cmd_enable_feedback(BaseSequentialStream *chp, int argc, char *argv[]) {
    if (argc != 1) {
        shellUsage(chp, "fb_enable feedback_id(0-9)");
        return;
    }
    unsigned feedback_id = Shell::atoi(argv[0]);
    if (feedback_id > 9) {
        chprintf(chp, "Invalid feedback ID %d" SHELL_NEWLINE_STR, feedback_id);
        return;
    }
    feedback_enable[feedback_id] = true;
    chprintf(chp, "%s feedback enabled" SHELL_NEWLINE_STR, motor_name[feedback_id]);
}

void cmd_disable_feedback(BaseSequentialStream *chp, int argc, char *argv[]) {
    if (argc != 1) {
        shellUsage(chp, "fb_disable feedback_id(0-9)");
        return;
    }
    unsigned feedback_id = Shell::atoi(argv[0]);
    if (feedback_id > 9) {
        chprintf(chp, "Invalid feedback ID %d" SHELL_NEWLINE_STR, feedback_id);
        return;
    }
    feedback_enable[feedback_id] = false;
    chprintf(chp, "%s feedback disabled" SHELL_NEWLINE_STR, motor_name[feedback_id]);
}

void cmd_set_param(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 7) {
        shellUsage(chp, "set_pid motor_id(0-5) pid_id(0: angle_to_v, 1: v_to_i) ki kp kd i_limit out_limit");
        return;
    }
    unsigned motor_id = Shell::atoi(argv[0]);
    if (motor_id > 5) {
        chprintf(chp, "Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return;
    }
    unsigned pid_id = Shell::atoi(argv[1]);
    if (pid_id > 1) {
        chprintf(chp, "Invalid pid ID %d" SHELL_NEWLINE_STR, pid_id);
        return;
    }
    PIDController::pid_params_t pid_param = {Shell::atof(argv[2]),
                                             Shell::atof(argv[3]),
                                             Shell::atof(argv[4]),
                                             Shell::atof(argv[5]),
                                             Shell::atof(argv[6])};

    if (motor_id < 3) {
        GimbalSKD::load_pid_params_by_type(pid_param, (GimbalBase::motor_id_t) motor_id, pid_id == 0);
    } else {
        ShootSKD::load_pid_params_by_type(pid_param, motor_id - 3, pid_id == 0);
    }
    chprintf(chp, "ps!" SHELL_NEWLINE_STR);
}

void cmd_echo_param(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "echo_pid motor_id(0-5) pid_id(0: angle_to_v, 1: v_to_i)");
        return;
    }
    unsigned motor_id = Shell::atoi(argv[0]);
    if (motor_id > 5) {
        chprintf(chp, "Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return;
    }
    unsigned pid_id = Shell::atoi(argv[1]);
    if (pid_id > 1) {
        chprintf(chp, "Invalid pid ID %d" SHELL_NEWLINE_STR, pid_id);
        return;
    }

    PIDController::pid_params_t pid_param = {0,0,0,0,0};
    if (motor_id < 3) {
        pid_param = GimbalSKD::echo_pid_params_by_type((GimbalBase::motor_id_t) motor_id, pid_id == 0);
    } else {
        pid_param = ShootSKD::echo_pid_params_by_type(motor_id - 3, pid_id == 0);
    }
    chprintf(chp, "ki: %.2f, kp: %.2f, kd: %.2f, i_limit: %.2f, out_limit: %.2f" SHELL_NEWLINE_STR,
                 pid_param.ki, pid_param.kp, pid_param.kd, pid_param.i_limit, pid_param.out_limit);
}

void cmd_set_target_angle(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "set_target_angle motor_id(0-3) angle");
        return;
    }
    unsigned motor_id = Shell::atoi(argv[0]);
    if (motor_id > 3) {
        chprintf(chp, "Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return;
    }
    float angle = Shell::atof(argv[1]);
    if (motor_id < 3) {
        float yaw_target = motor_id == GimbalBase::YAW ? angle : GimbalSKD::get_target_angle(GimbalBase::YAW);
        float pitch_target = motor_id == GimbalBase::PITCH ? angle : GimbalSKD::get_target_angle(GimbalBase::PITCH);
        float sub_pitch_target = motor_id == GimbalBase::SUB_PITCH ? angle : GimbalSKD::get_target_angle(GimbalBase::SUB_PITCH);
        GimbalSKD::set_target_angle(yaw_target, pitch_target, sub_pitch_target);
    } else {
        ShootSKD::reset_loader_accumulated_angle();
        ShootSKD::set_loader_target_angle(angle);
    }

    chprintf(chp, "ps!" SHELL_NEWLINE_STR);
}

void cmd_echo_target_angle(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "echo_target_angle motor_id(0-3)");
        return;
    }
    unsigned motor_id = Shell::atoi(argv[0]);
    if (motor_id > 3) {
        chprintf(chp, "Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return;
    }
    float angle = 0;
    if (motor_id < 3) {
        angle = GimbalSKD::get_target_angle((GimbalBase::motor_id_t) motor_id);
    } else {
        angle = ShootSKD::get_loader_target_angle();
    }
    chprintf(chp, "%.2f" SHELL_NEWLINE_STR, angle);
}

void cmd_echo_actual_angle(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "echo_actual_angle motor_id(0-3)");
        return;
    }
    unsigned motor_id = Shell::atoi(argv[0]);
    if (motor_id > 3) {
        chprintf(chp, "Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return;
    }
    float angle = 0;
    if (motor_id < 3) {
        angle = GimbalSKD::get_accumulated_angle((GimbalBase::motor_id_t) motor_id);
    } else {
        angle = ShootSKD::get_loader_accumulated_angle();
    }
    chprintf(chp, "%.2f" SHELL_NEWLINE_STR, angle);
}

void cmd_echo_raw_angle(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "echo_raw_angle motor_id(0-3)");
        return;
    }
    unsigned motor_id = Shell::atoi(argv[0]);
    if (motor_id > 3) {
        chprintf(chp, "Invalid motor ID %d" SHELL_NEWLINE_STR, motor_id);
        return;
    }
    chprintf(chp, "%d" SHELL_NEWLINE_STR, GimbalIF::feedback[motor_id]->last_angle_raw);
}

class FeedbackThread : public BaseStaticThread<512> {
private:
    void main() final {
        setName("Feedback");
        while (!shouldTerminate()) {
            float actual_angle, target_angle, actual_velocity, target_velocity;

            // Gimbal
            for (int i = 0; i < 3; i++) {
                if (feedback_enable[i]) {
                    actual_angle = GimbalSKD::get_accumulated_angle((GimbalBase::motor_id_t) i);
                    target_angle = GimbalSKD::get_target_angle((GimbalBase::motor_id_t) i);
                    actual_velocity = GimbalSKD::get_actual_velocity((GimbalBase::motor_id_t) i);
                    target_velocity = GimbalSKD::get_target_velocity((GimbalBase::motor_id_t) i);
                    Shell::printf("fb %s %.2f %.2f %.2f %.2f %d %d" SHELL_NEWLINE_STR,
                                  motor_name[i],
                                  actual_angle, target_angle,
                                  actual_velocity, target_velocity,
                                  GimbalIF::feedback[i]->actual_current, *GimbalIF::target_current[i]);
                }
            }

            if (feedback_enable[3])
                Shell::printf("fb %s %.2f 0 %.2f 0 %.2f 0" SHELL_NEWLINE_STR,
                              motor_name[3],
                              ShootSKD::get_loader_accumulated_angle(), ShootSKD::get_loader_target_angle(),
                              ShootSKD::get_actual_velocity(0), ShootSKD::get_target_velocity(0),
                              GimbalIF::feedback[GimbalBase::BULLET]->actual_current, ShootSKD::get_target_current(0));

            for (int i = 4; i <= 5; i++) {
                if (feedback_enable[i])
                    Shell::printf("fb %s %.2f 0 %.2f 0 %.2f 0" SHELL_NEWLINE_STR,
                                  motor_name[i],
                                  0, 0,
                                  ShootSKD::get_actual_velocity(i-3), ShootSKD::get_actual_velocity(i-3),
                                  GimbalIF::feedback[i]->actual_current, ShootSKD::get_target_current(i-3));
            }

            // AHRS
            if (feedback_enable[6])
                Shell::printf("fb %s %.2f 0 %.2f 0 %.2f 0" SHELL_NEWLINE_STR,
                              motor_name[6], ahrs.get_angle().x, ahrs.get_angle().y, ahrs.get_angle().z);
            if (feedback_enable[7])
                Shell::printf("fb %s %.2f 0 %.2f 0 %.2f 0" SHELL_NEWLINE_STR,
                              motor_name[7], ahrs.get_gyro().x, ahrs.get_gyro().y, ahrs.get_gyro().z);
            if (feedback_enable[8])
                Shell::printf("fb %s %.2f 0 %.2f 0 %.2f 0" SHELL_NEWLINE_STR,
                              motor_name[8], ahrs.get_accel().x, ahrs.get_accel().y, ahrs.get_accel().z);
            if (feedback_enable[9])
                Shell::printf("fb %s %.2f 0 %.2f 0 %.2f 0" SHELL_NEWLINE_STR,
                              motor_name[9], ahrs.get_magnet().x, ahrs.get_magnet().y, ahrs.get_magnet().z);

            sleep(TIME_MS2I(GIMBAL_FEEDBACK_INTERVAL));
        }
    }
} feedbackThread;

constexpr unsigned USER_THREAD_INTERVAL = 7;  // [ms]

float gimbal_yaw_target_angle_ = 0;
float gimbal_rc_yaw_max_speed = 180;  // [degree/s]

float gimbal_pitch_min_angle = -30; // down range for pitch [degree]
float gimbal_pitch_max_angle = 10; //  up range for pitch [degree]

bool is_shooting = false;
float bullet_velocity = 0;
float shoot_common_duty_cycle = 0.4;


class RemoteThread : public BaseStaticThread<512> {
private:
    void main() final {
        setName("Remote");
        while (!shouldTerminate()) {
            if (Remote::rc.s1 == Remote::S_MIDDLE) {

                /// Remote - Yaw + Pitch
                GimbalSKD::set_mode(GimbalSKD::ABS_ANGLE_MODE);

                gimbal_yaw_target_angle_ +=
                        -Remote::rc.ch0 * (gimbal_rc_yaw_max_speed * USER_THREAD_INTERVAL / 1000.0f);
                // ch0 use right as positive direction, while GimbalLG use CCW (left) as positive direction

                float pitch_target;
                if (Remote::rc.ch1 > 0) pitch_target = Remote::rc.ch1 * gimbal_pitch_max_angle * 0.1;
                else pitch_target = Remote::rc.ch1 * gimbal_pitch_min_angle * 0.1;  // GIMBAL_PITCH_MIN_ANGLE is negative
                // ch1 use up as positive direction, while GimbalLG also use up as positive direction

                VAL_CROP(pitch_target, gimbal_pitch_max_angle, gimbal_pitch_min_angle);
                GimbalSKD::set_target_angle(gimbal_yaw_target_angle_, pitch_target, 0);

            } else {
                /// Safe Mode
                GimbalSKD::set_mode(GimbalSKD::FORCED_RELAX_MODE);
            }


            /*** ---------------------------------- Shoot --------------------------------- ***/

            if (Remote::rc.s1 == Remote::S_MIDDLE) {

                /// Remote - Shoot with Scrolling Wheel

                ShootSKD::set_mode(ShootSKD::LIMITED_SHOOTING_MODE);

                if (Remote::rc.wheel > 0.5) {  // down
                    if (!is_shooting){
                        is_shooting = true;
                        ShootSKD::reset_loader_accumulated_angle();
                        ShootSKD::set_loader_target_velocity(bullet_velocity);
                    }
                    ShootSKD::set_loader_target_angle(GimbalIF::feedback[GimbalBase::BULLET]->accumulated_angle() + 600);
                } else if (Remote::rc.wheel < -0.5) {  // up
                    if (!is_shooting){
                        is_shooting = true;
                        ShootSKD::reset_loader_accumulated_angle();
                        ShootSKD::set_loader_target_velocity(bullet_velocity);
                    }
                    ShootSKD::set_loader_target_angle(GimbalIF::feedback[GimbalBase::BULLET]->accumulated_angle() - 600);
                } else {
                    if (is_shooting){
                        is_shooting = false;
                        ShootSKD::reset_loader_accumulated_angle();
                        ShootSKD::set_loader_target_angle(0);
                    }
                }

                ShootSKD::set_friction_wheels(shoot_common_duty_cycle);

            } else {
                /// Safe Mode
                ShootSKD::set_mode(ShootSKD::FORCED_RELAX_MODE);
                ShootSKD::set_friction_wheels(0);
            }

            /// Final
            sleep(TIME_MS2I(USER_THREAD_INTERVAL));
        }
    }
} remoteThread;

// Command lists for gimbal controller test and adjustments
ShellCommand mainProgramCommands[] = {
        {"set_enable",          cmd_set_enable},
        {"set_disable",         cmd_set_disable},
        {"get_sid",             cmd_get_sid},
        {"fb_enable",           cmd_enable_feedback},
        {"fb_disable",          cmd_disable_feedback},
        {"set_pid",             cmd_set_param},
        {"echo_pid",            cmd_echo_param},
        {"set_target_angle",    cmd_set_target_angle},
        {"echo_target_angle",   cmd_echo_target_angle},
        {"echo_actual_angle",   cmd_echo_actual_angle},
        {"echo_raw_angle",      cmd_echo_raw_angle},
        {nullptr,         nullptr}
};




int main(void) {
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
    /// Setup SDCard
    if (SDCard::init()) {
        SDCard::read_all();
        LED::led_on(DEV_BOARD_LED_SD_CARD);  // LED 8 on if SD card inserted
    }

    /// Setup CAN1 & CAN2
    can1.start(THREAD_CAN1_RX_PRIO, THREAD_CAN1_TX_PRIO);
    can2.start(THREAD_CAN2_RX_PRIO, THREAD_CAN2_TX_PRIO);
    chThdSleepMilliseconds(5);

    /// Start Feedback Thread
    feedbackThread.start(THREAD_FEEDBACK_PRIO);

    /// Setup On-Board AHRS
    Vector3D ahrs_bias;
    if (SDCard::get_data(MPU6500_BIAS_DATA_ID, &ahrs_bias, sizeof(ahrs_bias)) == SDCard::OK) {
        ahrs.load_calibration_data(ahrs_bias);
        LOG("Use AHRS bias in SD Card");
    } else {
        ahrs.load_calibration_data(MPU6500_STORED_GYRO_BIAS);
        LOG_WARN("Use default AHRS bias");
    }
    ahrs.start(ON_BOARD_AHRS_MATRIX_, THREAD_MPU_PRIO, THREAD_IST_PRIO, THREAD_AHRS_PRIO);
    chThdSleepMilliseconds(5);

    /// Setup Remote
    Remote::start();


    /// Setup GimbalIF (for Gimbal and Shoot)
    GimbalIF::init(&can1, &can2, GIMBAL_MOTOR_CONFIG_, GIMBAL_YAW_FRONT_ANGLE_RAW, GIMBAL_PITCH_FRONT_ANGLE_RAW, GIMBAL_SUB_PITCH_FRONT_ANGLE_RAW);
    chThdSleepMilliseconds(2000);  // wait for C610 to be online and friction wheel to reset


    /// Setup Red Spot Laser
    palSetPad(GPIOG, GPIOG_RED_SPOT_LASER);  // enable the red spot laser


    /*** ------------ Period 2. Calibration and Start Logic Control Thread ----------- ***/

    /// Echo Gimbal Raws and Converted Angles
    LOG("Gimbal Yaw: %u, %f, Pitch: %u, %f, Sub_Pitch: %u, %f",
        GimbalIF::feedback[GimbalIF::YAW]->last_angle_raw, GimbalIF::feedback[GimbalIF::YAW]->actual_angle,
        GimbalIF::feedback[GimbalIF::PITCH]->last_angle_raw, GimbalIF::feedback[GimbalIF::PITCH]->actual_angle,
        GimbalIF::feedback[GimbalIF::SUB_PITCH]->last_angle_raw, GimbalIF::feedback[GimbalIF::SUB_PITCH]->actual_angle);

    /// Start SKDs
    GimbalSKD::set_test_status(true);
    GimbalSKD::start(&ahrs, GIMBAL_ANGLE_INSTALLATION_MATRIX_, GIMBAL_GYRO_INSTALLATION_MATRIX_,
                     GIMBAL_YAW_INSTALL_DIRECTION, GIMBAL_PITCH_INSTALL_DIRECTION, GIMBAL_SUB_PITCH_INSTALL_DIRECTION, THREAD_GIMBAL_SKD_PRIO);
    GimbalSKD::load_pid_params(GIMBAL_PID_YAW_A2V_PARAMS, GIMBAL_PID_YAW_V2I_PARAMS,
                               GIMBAL_PID_PITCH_A2V_PARAMS, GIMBAL_PID_PITCH_V2I_PARAMS,
                               GIMBAL_PID_SUB_PITCH_A2V_PARAMS, GIMBAL_PID_SUB_PITCH_V2I_PARAMS);
//    GimbalSKD::set_yaw_restriction(GIMBAL_RESTRICT_YAW_MIN_ANGLE, GIMBAL_RESTRICT_YAW_MAX_ANGLE,
//                                   GIMBAL_RESTRICT_YAW_VELOCITY);

    ShootSKD::set_test_status(true);
    ShootSKD::start(SHOOT_BULLET_INSTALL_DIRECTION, THREAD_SHOOT_SKD_PRIO);
    ShootSKD::load_pid_params(SHOOT_PID_BULLET_LOADER_A2V_PARAMS, SHOOT_PID_BULLET_LOADER_V2I_PARAMS,
                              SHOOT_PID_FW_LEFT_V2I_PARAMS, SHOOT_PID_FW_RIGHT_V2I_PARAMS);
    remoteThread.start(THREAD_USER_PRIO);

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