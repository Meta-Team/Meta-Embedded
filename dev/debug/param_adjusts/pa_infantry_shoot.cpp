//
// Created by ... on YYYY/MM/DD.
//

/**
 * This file contain ... Unit Test.
 */

#include "ch.hpp"
#include "hal.h"

#include "ahrs.h"
#include "sd_card_interface.h"

#include "remote_interpreter.h"

#include "led.h"
#include "debug/shell/shell.h"
#include "interface/gimbal_interface.h"
#include "module/pid_controller.hpp"
#include "vehicle/infantry/vehicle_infantry.h"
#include "scheduler/gimbal_scheduler.h"

#include "scheduler/buzzer_scheduler.h"

#define MPU6500_STORED_GYRO_BIAS {1.142140388f, -1.022603631f, 0.125763729f}
// Other headers here

using namespace chibios_rt;

CANInterface::motor_feedback_t *feedback[4];
int *targetCurrent[2];
float loaderV;

PIDController::pid_params_t upper_pid;
PIDController::pid_params_t lower_pid;

PIDController v2iController[2];
PIDController loaderPID;

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);
AHRSOnBoard ahrs;

float targetVelocity[2];

bool enable_u_feedback;
bool enable_l_feedback;

unsigned const GIMBAL_THREAD_INTERVAL = 1;    // [ms]
unsigned const GIMBAL_FEEDBACK_INTERVAL = 25; // [ms]

static const Matrix33 ON_BOARD_AHRS_MATRIX_ = ON_BOARD_AHRS_MATRIX;
static const Matrix33 GIMBAL_ANGLE_INSTALLATION_MATRIX_ = GIMBAL_ANGLE_INSTALLATION_MATRIX;
static const Matrix33 GIMBAL_GYRO_INSTALLATION_MATRIX_ = GIMBAL_GYRO_INSTALLATION_MATRIX;

static GimbalIF::motor_can_config_t GIMBAL_MOTOR_CONFIG_[GimbalIF::MOTOR_COUNT] = GIMBAL_MOTOR_CONFIG;

#define GIMBAL_YAW_FRONT_ANGLE_RAW 3337
#define GIMBAL_PITCH_FRONT_ANGLE_RAW 2866  // of no use now

/**
 * @brief set enabled state of yaw and pitch motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_gimbal_enable_feedback(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2 || (*argv[0] != '0' && *argv[0] != '1') || (*argv[1] != '0' && *argv[1] != '1')) {
        shellUsage(chp, "g_enable_fb yaw(0/1) pitch(0/1)");
        return;
    }
    enable_l_feedback = *argv[0] - '0';
    enable_u_feedback = *argv[1] - '0';
}

static void cmd_set_target(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "g_set_v yaw_velocity pitch_velocity");
        return;
    }
    targetVelocity[0] = Shell::atof(argv[0]);
    targetVelocity[1] = Shell::atof(argv[1]);
}

static void cmd_set_pid(BaseSequentialStream *chp, int argc, char **argv) {
    (void) argv;
    if (argc != 7) {
        shellUsage(chp, "g_set_params yaw(0)/pitch(1) angle_to_v(0)/v_to_i(1) ki kp kd i_limit out_limit");
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }
    PIDController *p;
    if (*argv[0] == '0') p = &v2iController[0];
    else if (*argv[0] == '1' ) p = &v2iController[1];
    else {
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }

    p->change_parameters({Shell::atof(argv[2]),
                          Shell::atof(argv[3]),
                          Shell::atof(argv[4]),
                          Shell::atof(argv[5]),
                          Shell::atof(argv[6])});

    chprintf(chp, "!ps" SHELL_NEWLINE_STR); // echo parameters set
}

// Shell commands to ...
ShellCommand ShellCommands[] = {
        {"g_set_v",      cmd_set_target},
        {"g_set_params", cmd_set_pid},
        {"g_enable_fb",  cmd_gimbal_enable_feedback},
        {nullptr,        nullptr}
};


// Thread to ...
class SKDThread : public BaseStaticThread <512> {
private:
    void main() final {

        setName("SKDThread");
        while (!shouldTerminate()) {
            if (!ABS_IN_RANGE(Remote::rc.wheel,0.2)) {
                loaderV = 100.0f;
                targetVelocity[0] = targetVelocity[1] = Remote::rc.wheel * 2800.0f;
            } else {
                loaderV = 0.0f;
                targetVelocity[0] = targetVelocity[1] = 0.0f;
            }
            GimbalSKD::set_target_angle(0.0f, 0.0f);
            *GimbalIF::target_current[GimbalIF::FW_LEFT]= (int)v2iController[0].calc(GimbalIF::feedback[GimbalIF::FW_LEFT]->actual_velocity, targetVelocity[0]);
            *GimbalIF::target_current[GimbalIF::FW_RIGHT]= (int)v2iController[1].calc(GimbalIF::feedback[GimbalIF::FW_RIGHT]->actual_velocity, -targetVelocity[0]);
            *GimbalIF::target_current[GimbalIF::BULLET] = (int)loaderPID.calc(GimbalIF::feedback[GimbalIF::BULLET]->actual_velocity, loaderV);
            sleep(TIME_MS2I(5));
        }
    }
} skdThread;

class FeedbackThread : public BaseStaticThread<512> {
private:
    void main() final {
        setName("feedback");
        while (!shouldTerminate()) {
            if (enable_l_feedback) {
                Shell::printf("!gy,%u,%.2f,%.2f,%.2f,%.2f,%d,%d" SHELL_NEWLINE_STR,
                              SYSTIME,
                              GimbalIF::feedback[GimbalIF::FW_LEFT]->actual_velocity, targetVelocity[0],
                              GimbalIF::feedback[GimbalIF::FW_LEFT]->actual_velocity, targetVelocity[0],
                              GimbalIF::feedback[GimbalIF::FW_LEFT]->actual_current,  targetCurrent[0]);
            }
            if (enable_u_feedback) {
                Shell::printf("!gp,%u,%.2f,%.2f,%.2f,%.2f,%d,%d" SHELL_NEWLINE_STR,
                              SYSTIME,
                              GimbalIF::feedback[GimbalIF::FW_RIGHT]->actual_velocity, targetVelocity[1],
                              GimbalIF::feedback[GimbalIF::FW_RIGHT]->actual_velocity, targetVelocity[1],
                              GimbalIF::feedback[GimbalIF::FW_RIGHT]->actual_current,  targetCurrent[1]);
            }
            sleep(TIME_MS2I(GIMBAL_FEEDBACK_INTERVAL));
        }
    }
} feedbackThread;

int main(void) {
    halInit();
    System::init();

    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    Shell::addCommands(ShellCommands);

    can1.start(HIGHPRIO - 6, HIGHPRIO - 7);
    can2.start(HIGHPRIO - 8, HIGHPRIO - 9);

    Remote::start();

    /*** Set on board ahrs ***/
    Vector3D ahrs_bias;
    if (SDCard::get_data(MPU6500_BIAS_DATA_ID, &ahrs_bias, sizeof(ahrs_bias)) == SDCard::OK) {
        ahrs.load_calibration_data(ahrs_bias);
        LOG("Use AHRS bias in SD Card");
    } else {
        ahrs.load_calibration_data(MPU6500_STORED_GYRO_BIAS);
        LOG_WARN("Use default AHRS bias");
    }
    ahrs.load_calibration_data({-0.644649505f, -0.619945943f, 0.173617705f});
    ahrs.start(ON_BOARD_AHRS_MATRIX_, THREAD_MPU_PRIO);

    GimbalIF::init(&can1, &can2, GIMBAL_MOTOR_CONFIG_, GIMBAL_YAW_FRONT_ANGLE_RAW, GIMBAL_PITCH_FRONT_ANGLE_RAW);
    chThdSleepMilliseconds(2000);  // wait for C610 to be online and friction wheel to reset

    GimbalSKD::start(&ahrs, GIMBAL_ANGLE_INSTALLATION_MATRIX_, GIMBAL_GYRO_INSTALLATION_MATRIX_,
                     GIMBAL_YAW_INSTALL_DIRECTION, GIMBAL_PITCH_INSTALL_DIRECTION, THREAD_GIMBAL_SKD_PRIO);
    GimbalSKD::load_pid_params(GIMBAL_PID_YAW_A2V_PARAMS, GIMBAL_PID_YAW_V2I_PARAMS,
                               GIMBAL_PID_PITCH_A2V_PARAMS, GIMBAL_PID_PITCH_V2I_PARAMS);
    GimbalSKD::set_yaw_restriction(GIMBAL_RESTRICT_YAW_MIN_ANGLE, GIMBAL_RESTRICT_YAW_MAX_ANGLE,
                                   GIMBAL_RESTRICT_YAW_VELOCITY);

    GimbalSKD::set_mode(GimbalSKD::ABS_ANGLE_MODE);
    skdThread.start(NORMALPRIO + 1);
    feedbackThread.start(NORMALPRIO + 2);

    v2iController[0].change_parameters(SHOOT_PID_FW_LEFT_V2I_PARAMS);
    v2iController[1].change_parameters(SHOOT_PID_FW_RIGHT_V2I_PARAMS);
    loaderPID.change_parameters(SHOOT_PID_BULLET_LOADER_V2I_PARAMS);

    BuzzerSKD::init(NORMALPRIO + 4);
    BuzzerSKD::play_sound(BuzzerSKD::sound_nyan_cat);

#if CH_CFG_NO_IDLE_THREAD // see chconf.h for what this #define means
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    BaseThread::setPriority(1);
#endif
    return 0;
}
