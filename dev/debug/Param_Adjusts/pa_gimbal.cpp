//
// Created by liuzikai on 2019-01-07.
//

/**
 * This file contain program for GimbalParameter adjustment for Yaw and Pitch motors.
 */

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "debug/shell/shell.h"

#include "can_interface.h"
#include "ahrs.h"
#include "sd_card_interface.h"

#include "buzzer_scheduler.h"

#include "scheduler/gimbal_scheduler.h"
#include "scheduler/shoot_scheduler.h"

using namespace chibios_rt;

// Duplicate of motor_id_t in GimbalIF to reduce code
unsigned const YAW = GimbalIF::YAW;
unsigned const PITCH = GimbalIF::PITCH;

char MOTOR_CHAR[2] = {'y', 'p'};

// Calculation interval for gimbal thread
unsigned const GIMBAL_THREAD_INTERVAL = 1;    // [ms]
unsigned const GIMBAL_FEEDBACK_INTERVAL = 25; // [ms]

float const MIN_ANGLE[2] = {-170, -80};    // [degree]
float const MAX_ANGLE[2] = {170, 80};      // [degree]
float const MAX_VELOCITY[2] = {1000, 1000};  // absolute maximum, [degree/s]
int const MAX_CURRENT = 20000;  // [mA]

bool motor_enabled[2] = {false, false};

bool enable_a2v_pid = false;
// If not enabled, the thread will take target_velocity and perform v_to_i convention.
// If enabled, the thread will take target_angle and perform two-ring conventions.

float target_angle[2] = {0.0, 0.0};
float target_v[2] = {0.0, 0.0};

// Raw angle of yaw and pitch when GimbalIF points straight forward.
//   Note: the program will echo the raw angles of yaw and pitch as the program starts
#define GIMBAL_YAW_FRONT_ANGLE_RAW 5372
#define GIMBAL_PITCH_FRONT_ANGLE_RAW 4128

// INFANTRY AHRS PARAMETERS
// Depends on the install direction of the board
#define ON_BOARD_AHRS_MATRIX {{0.0f, -1.0f, 0.0f}, \
                              {1.0f, 0.0f, 0.0f}, \
                              {0.0f, 0.0f, 1.0f}}

#define GIMBAL_ANGLE_INSTALLATION_MATRIX {{1.0f, 0.0f, 0.0f}, \
                                          {0.0f, 1.0f, 0.0f}, \
                                          {0.0f, 0.0f, -1.0f}}


#define GIMBAL_GYRO_INSTALLATION_MATRIX {{0.0f,  1.0f, 0.0f}, \
                                         {0.0f,  0.0f,  -1.0f}, \
                                         {-1.0f, 0.0f,  0.0f}}

static const Matrix33 ON_BOARD_AHRS_MATRIX_ = ON_BOARD_AHRS_MATRIX;
static const Matrix33 GIMBAL_ANGLE_INSTALLATION_MATRIX_ = GIMBAL_ANGLE_INSTALLATION_MATRIX;
static const Matrix33 GIMBAL_GYRO_INSTALLATION_MATRIX_ = GIMBAL_GYRO_INSTALLATION_MATRIX;

#define MPU6500_BIAS_DATA_ID 0x0001
// INFANTRY FOUR
#define MPU6500_STORED_GYRO_BIAS {1.142140388f, -1.022603631f, 0.125763729f}

#define THREAD_CAN1_PRIO                    (HIGHPRIO - 1)
#define THREAD_MPU_PRIO                     (HIGHPRIO - 2)
#define THREAD_IST_PRIO                     (HIGHPRIO - 3)
#define THREAD_AHRS_PRIO                    (HIGHPRIO - 4)

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);
AHRSOnBoard ahrs;

bool ahrs_enabled;

class GimbalDebugThread : public BaseStaticThread<1024> {
public:
    PIDController a2v_pid[2];
    PIDController v2i_pid[2];

    float actual_angle[2];

    float target_velocity[2];
    float actual_velocity[2];

protected:
    void main() final {
        setName("gimbal");
        while (!shouldTerminate()) {
            if(ahrs_enabled) {
                // AHRS calculation
                Vector3D ahrs_angle = ahrs.get_angle() * GIMBAL_ANGLE_INSTALLATION_MATRIX_;
                Vector3D ahrs_gyro = ahrs.get_gyro() * GIMBAL_GYRO_INSTALLATION_MATRIX_;

                // TODO: document calculations here
                actual_angle[0] = ahrs_angle.x;
                actual_angle[1] = ahrs_angle.y;
                float velocity_[2] = {
                        ahrs_gyro.x * cosf(ahrs_angle.y / 180.0f * M_PI) + ahrs_gyro.z * sinf(ahrs_angle.y / 180.0f * M_PI),
                        ahrs_gyro.y};
                actual_velocity[0] = velocity_[0];
                actual_velocity[1] = velocity_[1];
            } else {
                actual_angle[0] = GimbalIF::feedback[YAW]->actual_angle;
                actual_angle[1] = GimbalIF::feedback[PITCH]->actual_angle;

                actual_velocity[0] = GimbalIF::feedback[YAW]->actual_velocity;
                actual_velocity[1] = GimbalIF::feedback[PITCH]->actual_velocity;
            }

            // Calculation and check
            if (motor_enabled[YAW] || motor_enabled[PITCH]) {

                for (unsigned i = YAW; i <= PITCH; i++) {

                    // Perform angle check
//                    if (GimbalIF::feedback[i]->actual_angle > MAX_ANGLE[i]) {
//                        Shell::printf("!d%cA" SHELL_NEWLINE_STR, MOTOR_CHAR[i]);
//                        motor_enabled[i] = false;
//                        continue;
//                    }
//                    if (GimbalIF::feedback[i]->actual_angle < MIN_ANGLE[i]) {
//                        Shell::printf("!d%ca" SHELL_NEWLINE_STR, MOTOR_CHAR[i]);
//                        motor_enabled[i] = false;
//                        continue;
//                    }

                        if (enable_a2v_pid) {
                            // Calculate from angle to velocity
                            target_velocity[i] = a2v_pid[i].calc(actual_angle[i],target_angle[i]);
                        } else {
                            // Directly fill the target velocity
                            target_velocity[i] = target_v[i];
                        }

                    // Perform velocity check

//                    if (actual_velocity[i] > MAX_VELOCITY[i]) {
//                        Shell::printf("!d%cv" SHELL_NEWLINE_STR, MOTOR_CHAR[i]);
//                        motor_enabled[i] = false;
//                        continue;
//                    }

                    // Calculate from velocity to current
                    *GimbalIF::target_current[i] = (int) v2i_pid[i].calc(actual_velocity[i], target_velocity[i]);
                    // NOTE: Gimbal::target_velocity[i] is either calculated or filled (see above)


                    // Perform current check
                    if (*GimbalIF::target_current[i] > MAX_CURRENT || *GimbalIF::target_current[i] < -MAX_CURRENT) {
                        Shell::printf("!d%cc" SHELL_NEWLINE_STR, MOTOR_CHAR[i]);
                        motor_enabled[i] = false;
                        continue;
                    }
                }

            }

            // This two operations should be after calculation since motor can get disabled if check failed
            // This two operations should always perform, instead of being put in a 'else' block
            if (!motor_enabled[YAW]) *GimbalIF::target_current[YAW] = 0;
            if (!motor_enabled[PITCH]) *GimbalIF::target_current[PITCH] = 0;

            // Send currents
//            GimbalIF::enable_gimbal_current_clip();

            sleep(TIME_MS2I(GIMBAL_THREAD_INTERVAL));
        }
    }
} gimbalThread;

class GimbalFeedbackThread : public chibios_rt::BaseStaticThread<1024> {

public:

    bool enable_yaw_feedback = false;
    bool enable_pitch_feedback = false;

private:

    void main() final {

        setName("gimbal_fb");

        while (!shouldTerminate()) {

            if (enable_yaw_feedback) {
                Shell::printf("!gy,%u,%.2f,%.2f,%.2f,%.2f,%d,%d" SHELL_NEWLINE_STR,
                              SYSTIME,
                              gimbalThread.actual_angle[YAW], target_angle[YAW],
                              gimbalThread.actual_velocity[YAW], target_v[YAW],
                              GimbalIF::feedback[YAW]->actual_current, *GimbalIF::target_current[YAW]);
            }
            if (enable_pitch_feedback) {
                Shell::printf("!gp,%u,%.2f,%.2f,%.2f,%.2f,%d,%d" SHELL_NEWLINE_STR,
                              SYSTIME,
                              gimbalThread.actual_angle[PITCH], target_angle[PITCH],
                              gimbalThread.actual_velocity[PITCH], target_v[PITCH],
                              GimbalIF::feedback[PITCH]->actual_current, *GimbalIF::target_current[PITCH]);
            }

            sleep(TIME_MS2I(GIMBAL_FEEDBACK_INTERVAL));
        }
    }

} gimbalFeedbackThread;

/**
 * @brief set enabled states of yaw and pitch motors
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_gimbal_enable(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2 || (*argv[0] != '0' && *argv[0] != '1') || (*argv[1] != '0' && *argv[1] != '1')) {
        shellUsage(chp, "g_enable yaw(0/1) pitch(0/1)");
        return;
    }
    motor_enabled[YAW] = *argv[0] - '0';
    motor_enabled[PITCH] = *argv[1] - '0';
}

/**
 * @brief set enabled state of friction wheels
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_gimbal_enable_fw(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1 || (*argv[0] != '0' && *argv[0] != '1')) {
        shellUsage(chp, "g_enable_fw 0/1");
        return;
    }
    if (*argv[0] == '1') {
        ShootSKD::set_friction_wheels(0.8);
    } else {
        ShootSKD::set_friction_wheels(0);
    }
//    GimbalIF::enable_gimbal_current_clip();
}

/**
 * @brief set feedback enable states
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
    gimbalFeedbackThread.enable_yaw_feedback = *argv[0] - '0';
    gimbalFeedbackThread.enable_pitch_feedback = *argv[1] - '0';
}


/**
 * @brief set front_angle_raw with current actual angle
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_gimbal_fix_front_angle(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "g_fix");
        return;
    }
    GimbalIF::feedback[YAW]->reset_front_angle();
    GimbalIF::feedback[PITCH]->reset_front_angle();

//    chprintf(chp, "!f" SHELL_NEWLINE_STR);
}

void _cmd_gimbal_clear_i_out() {
    for (int i = 0; i < 2; i++) {
        gimbalThread.v2i_pid[i].clear_i_out();
        gimbalThread.a2v_pid[i].clear_i_out();
    }
}

/**
 * @brief set target velocity of yaw and pitch and disable pos_to_v_pid
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_gimbal_set_target_velocities(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "g_set_v yaw_velocity pitch_velocity");
        return;
    }

    target_v[YAW] = Shell::atof(argv[0]);
    target_v[PITCH] = Shell::atof(argv[1]);
    _cmd_gimbal_clear_i_out();

    enable_a2v_pid = false;
}

/**
 * @brief set target angle of yaw and pitch and enable pos_to_v_pid
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_gimbal_set_target_angle(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "g_set_angle yaw_angle pitch_angle");
        return;
    }

    target_angle[YAW] = Shell::atof(argv[0]);
    target_angle[PITCH] = Shell::atof(argv[1]);
    _cmd_gimbal_clear_i_out();

    enable_a2v_pid = true;
}

/**
 * @brief set pid parameters
 * @param chp
 * @param argc
 * @param argv
 */
void cmd_gimbal_set_parameters(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 7) {
        shellUsage(chp, "g_set_params yaw(0)/pitch(1) angle_to_v(0)/v_to_i(1) ki kp kd i_limit out_limit");
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }
    PIDController *p;
    if (*argv[0] == '0' && *argv[1] == '0') p = &gimbalThread.a2v_pid[0];
            else if (*argv[0] == '0' && *argv[1] == '1') p = &gimbalThread.v2i_pid[0];
            else if (*argv[0] == '1' && *argv[1] == '0') p = &gimbalThread.a2v_pid[1];
            else if (*argv[0] == '1' && *argv[1] == '1') p = &gimbalThread.v2i_pid[1];
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

/**
 * @brief helper function for cmd_gimbal_echo_parameters()
 */
static inline void _cmd_gimbal_echo_parameters(BaseSequentialStream *chp, PIDController::pid_params_t p) {
    chprintf(chp, "%f %f %f %f %f" SHELL_NEWLINE_STR, p.kp, p.ki, p.kd, p.i_limit, p.out_limit);
}

/**
 * @brief echo pid parameters
 * @param chp
 * @param argc
 * @param argv
 */
void cmd_gimbal_echo_parameters(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "g_echo_params");
        return;
    }

    chprintf(chp, "yaw angle_to_v:   ");
    _cmd_gimbal_echo_parameters(chp, gimbalThread.a2v_pid[YAW].get_parameters());
    chprintf(chp, "yaw v_to_i:       ");
    _cmd_gimbal_echo_parameters(chp, gimbalThread.v2i_pid[YAW].get_parameters());
    chprintf(chp, "pitch angle_to_v: ");
    _cmd_gimbal_echo_parameters(chp, gimbalThread.a2v_pid[PITCH].get_parameters());
    chprintf(chp, "pitch v_to_i:     ");
    _cmd_gimbal_echo_parameters(chp, gimbalThread.v2i_pid[PITCH].get_parameters());
}

void cmd_enable_ahrs_feedback (BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "g_ahrs_e (0/1) (disable/enable)");
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }
    if(*argv[0] == '0' || *argv[0] == '1')ahrs_enabled = (bool) Shell::atoi(argv[0]);
}

// Command lists for gimbal controller test and adjustments
ShellCommand gimbalCotrollerCommands[] = {
        {"g_enable",      cmd_gimbal_enable},
        {"g_enable_fb",   cmd_gimbal_enable_feedback},
        {"g_fix",         cmd_gimbal_fix_front_angle},
        {"g_set_v",       cmd_gimbal_set_target_velocities},
        {"g_set_angle",   cmd_gimbal_set_target_angle},
        {"g_set_params",  cmd_gimbal_set_parameters},
        {"g_echo_params", cmd_gimbal_echo_parameters},
        {"g_enable_fw",   cmd_gimbal_enable_fw},
        {"g_ahrs_e",      cmd_enable_ahrs_feedback},
        {nullptr,         nullptr}
};

int main(void) {

    halInit();
    System::init();
    LED::all_off();
    Shell::start(HIGHPRIO-5);
    Shell::addCommands(gimbalCotrollerCommands);
    chThdSleepMilliseconds(400);
    can1.start(HIGHPRIO - 6, HIGHPRIO - 7);
    can2.start(HIGHPRIO - 8, HIGHPRIO - 9);

    BuzzerSKD::init(LOWPRIO +1);

    /// Setup On-Board AHRS
    Vector3D ahrs_bias;
    if (SDCard::get_data(MPU6500_BIAS_DATA_ID, &ahrs_bias, sizeof(ahrs_bias)) == SDCard::OK) {
        ahrs.load_calibration_data(ahrs_bias);
        LOG("Use AHRS bias in SD Card");
    } else {
        ahrs.load_calibration_data(MPU6500_STORED_GYRO_BIAS);
        LOG_WARN("Use default AHRS bias");
    }
    ahrs.load_calibration_data({-0.644649505f, -0.619945943f, 0.173617705f});
    ahrs.start(ON_BOARD_AHRS_MATRIX_, THREAD_MPU_PRIO, THREAD_IST_PRIO, THREAD_AHRS_PRIO);

    GimbalIF::motor_can_config_t canConfig[6] = {{GimbalIF::can_channel_2,4,CANInterface::GM6020},
                                                 {GimbalIF::can_channel_1,5,CANInterface::GM6020},
                                                 {GimbalIF::none_can_channel,6,CANInterface::NONE_MOTOR},
                                                 {GimbalIF::none_can_channel,8,CANInterface::NONE_MOTOR},
                                                 {GimbalIF::none_can_channel,9,CANInterface::NONE_MOTOR},
                                                 {GimbalIF::none_can_channel,10,CANInterface::NONE_MOTOR}};
    GimbalIF::init(&can1, &can2,
            canConfig,
            GIMBAL_YAW_FRONT_ANGLE_RAW, GIMBAL_PITCH_FRONT_ANGLE_RAW);

    gimbalFeedbackThread.start(NORMALPRIO - 1);
    gimbalThread.start(NORMALPRIO);

    //BuzzerSKD::play_sound(BuzzerSKD::sound_kong_fu_FC);

    chThdSleepMilliseconds(1000);
    LOG("Gimbal Yaw: %u, %f, Pitch: %u, %f",
        GimbalIF::feedback[GimbalIF::YAW]->last_angle_raw, GimbalIF::feedback[GimbalIF::YAW]->actual_angle,
        GimbalIF::feedback[GimbalIF::PITCH]->last_angle_raw, GimbalIF::feedback[GimbalIF::PITCH]->actual_angle);
    // See chconf.h for what this #define means.
#if CH_CFG_NO_IDLE_THREAD
    // ChibiOS idle thread has been disabled,
    // main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    BaseThread::setPriority(1);
#endif
    return 0;
}