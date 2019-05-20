//
// Created by liuzikai on 2019-01-07.
//

/**
 * This file contain program for GimbalParameter adjustment for Yaw and Pitch motors.
 */

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"

#include "can_interface.h"
#include "mpu6500.h"

#include "gimbal.h"
#include "shoot.h"

using namespace chibios_rt;

// Duplicate of motor_id_t in GimbalInterface to reduce code
unsigned const YAW = Gimbal::YAW;
unsigned const PITCH = Gimbal::PITCH;

char MOTOR_CHAR[2] = {'y', 'p'};

// Calculation interval for gimbal thread
unsigned const GIMBAL_THREAD_INTERVAL = 1;    // [ms]
unsigned const GIMBAL_FEEDBACK_INTERVAL = 25; // [ms]

float const MIN_ANGLE[2] = {-170, -80};    // [degree]
float const MAX_ANGLE[2] = {170, 80};      // [degree]
float const MAX_VELOCITY[2] = {600, 300};  // absolute maximum, [degree/s]
int const MAX_CURRENT = 30000;  // [mA]

bool motor_enabled[2] = {false, false};

bool enable_a2v_pid = false;
// If not enabled, the thread will take target_velocity and perform v_to_i convention.
// If enabled, the thread will take target_angle and perform two-ring conventions.

float target_angle[2] = {0.0, 0.0};
float target_v[2] = {0.0, 0.0};

// Raw angle of yaw and pitch when GimbalInterface points straight forward.
//   Note: the program will echo the raw angles of yaw and pitch as the program starts
#define GIMBAL_YAW_FRONT_ANGLE_RAW 5372
#define GIMBAL_PITCH_FRONT_ANGLE_RAW 4128

// Depends on the install direction of the board
#define GIMBAL_YAW_ACTUAL_VELOCITY (-MPU6500::angle_speed.z)
#define GIMBAL_PITCH_ACTUAL_VELOCITY (MPU6500::angle_speed.x)

CANInterface can1(&CAND1);


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
                              Gimbal::feedback[YAW].actual_angle, target_angle[YAW],
                              GIMBAL_YAW_ACTUAL_VELOCITY, target_v[YAW],
                              Gimbal::feedback[YAW].actual_current, Gimbal::target_current[YAW]);
            }
            if (enable_pitch_feedback) {
                Shell::printf("!gp,%u,%.2f,%.2f,%.2f,%.2f,%d,%d" SHELL_NEWLINE_STR,
                              SYSTIME,
                              Gimbal::feedback[PITCH].actual_angle, target_angle[PITCH],
                              GIMBAL_PITCH_ACTUAL_VELOCITY, target_v[PITCH],
                              Gimbal::feedback[PITCH].actual_current, Gimbal::target_current[PITCH]);
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
        Shoot::set_friction_wheels(0.8);
    } else {
        Shoot::set_friction_wheels(0);
    }
    Shoot::send_gimbal_currents();
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
    Gimbal::feedback[YAW].reset_front_angle();
    Gimbal::feedback[PITCH].reset_front_angle();

//    chprintf(chp, "!f" SHELL_NEWLINE_STR);
}

void _cmd_gimbal_clear_i_out() {
    for (int i = 0; i < 2; i++) {
        Gimbal::v2i_pid[i].clear_i_out();
        Gimbal::a2v_pid[i].clear_i_out();
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
        shellUsage(chp, "g_set_params yaw(0)/pitch(1) angle_to_v(0)/v_to_i(0) ki kp kd i_limit out_limit");
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }

    Gimbal::pid_params_t yaw_a2v_params = Gimbal::a2v_pid[YAW].get_parameters();
    Gimbal::pid_params_t yaw_v2i_params = Gimbal::v2i_pid[YAW].get_parameters();
    Gimbal::pid_params_t pitch_a2v_params = Gimbal::a2v_pid[PITCH].get_parameters();
    Gimbal::pid_params_t pitch_v2i_params = Gimbal::v2i_pid[PITCH].get_parameters();

    Gimbal::pid_params_t *p = nullptr;
    if (*argv[0] == '0' && *argv[1] == '0') p = &yaw_a2v_params;
    else if (*argv[0] == '0' && *argv[1] == '1') p = &yaw_v2i_params;
    else if (*argv[0] == '1' && *argv[1] == '0') p = &pitch_a2v_params;
    else if (*argv[0] == '1' && *argv[1] == '1') p = &pitch_v2i_params;
    else {
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }

    *p = {Shell::atof(argv[2]),
          Shell::atof(argv[3]),
          Shell::atof(argv[4]),
          Shell::atof(argv[5]),
          Shell::atof(argv[6])};

    Gimbal::change_pid_params(yaw_a2v_params, yaw_v2i_params, pitch_a2v_params, pitch_v2i_params);

    chprintf(chp, "!ps" SHELL_NEWLINE_STR); // echo parameters set
}

/**
 * @brief helper function for cmd_gimbal_echo_parameters()
 */
static inline void _cmd_gimbal_echo_parameters(BaseSequentialStream *chp, Gimbal::pid_params_t p) {
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
    _cmd_gimbal_echo_parameters(chp, Gimbal::a2v_pid[YAW].get_parameters());
    chprintf(chp, "yaw v_to_i:       ");
    _cmd_gimbal_echo_parameters(chp, Gimbal::v2i_pid[YAW].get_parameters());
    chprintf(chp, "pitch angle_to_v: ");
    _cmd_gimbal_echo_parameters(chp, Gimbal::a2v_pid[PITCH].get_parameters());
    chprintf(chp, "pitch v_to_i:     ");
    _cmd_gimbal_echo_parameters(chp, Gimbal::v2i_pid[PITCH].get_parameters());
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
        {nullptr,         nullptr}
};


class GimbalDebugThread : public BaseStaticThread<1024> {
protected:
    void main() final {
        setName("gimbal");
        while (!shouldTerminate()) {

            // Calculation and check
            if (motor_enabled[YAW] || motor_enabled[PITCH]) {

                for (unsigned i = YAW; i <= PITCH; i++) {

                    // Perform angle check
                    if (Gimbal::feedback[i].actual_angle > MAX_ANGLE[i]) {
                        Shell::printf("!d%cA" SHELL_NEWLINE_STR, MOTOR_CHAR[i]);
                        motor_enabled[i] = false;
                        continue;
                    }
                    if (Gimbal::feedback[i].actual_angle < MIN_ANGLE[i]) {
                        Shell::printf("!d%ca" SHELL_NEWLINE_STR, MOTOR_CHAR[i]);
                        motor_enabled[i] = false;
                        continue;
                    }

                    if (enable_a2v_pid) {
                        // Calculate from angle to velocity
                        Gimbal::calc_a2v_((Gimbal::motor_id_t) i, Gimbal::feedback[i].actual_angle, target_angle[i]);
                    } else {
                        // Directly fill the target velocity
                        Gimbal::target_velocity[i] = target_v[i];
                    }

                    // Perform velocity check
                    float actual_velocity_;
                    if (i == YAW) actual_velocity_ = GIMBAL_YAW_ACTUAL_VELOCITY;
                    else actual_velocity_ = GIMBAL_PITCH_ACTUAL_VELOCITY;
                    if (actual_velocity_ > MAX_VELOCITY[i]) {
                        Shell::printf("!d%cv" SHELL_NEWLINE_STR, MOTOR_CHAR[i]);
                        motor_enabled[i] = false;
                        continue;
                    }

                    // Calculate from velocity to current
                    Gimbal::calc_v2i_((Gimbal::motor_id_t) i, actual_velocity_, Gimbal::target_velocity[i]);
                    // NOTE: Gimbal::target_velocity[i] is either calculated or filled (see above)


                    // Perform current check
                    if (Gimbal::target_current[i] > MAX_CURRENT || Gimbal::target_current[i] < -MAX_CURRENT) {
                        Shell::printf("!d%cc" SHELL_NEWLINE_STR, MOTOR_CHAR[i]);
                        motor_enabled[i] = false;
                        continue;
                    }
                }

            }

            // This two operations should be after calculation since motor can get disabled if check failed
            // This two operations should always perform, instead of being put in a 'else' block
            if (!motor_enabled[YAW]) Gimbal::target_current[YAW] = 0;
            if (!motor_enabled[PITCH]) Gimbal::target_current[PITCH] = 0;

            // Send currents
            GimbalInterface::send_gimbal_currents();

            sleep(TIME_MS2I(GIMBAL_THREAD_INTERVAL));
        }
    }
} gimbalThread;


int main(void) {

    halInit();
    System::init();
    LED::all_off();
    Shell::start(HIGHPRIO);
    Shell::addCommands(gimbalCotrollerCommands);

    can1.start(HIGHPRIO - 1);
    MPU6500::start(HIGHPRIO - 2);
    chThdSleepMilliseconds(10);
    Gimbal::init(&can1, GIMBAL_YAW_FRONT_ANGLE_RAW, GIMBAL_PITCH_FRONT_ANGLE_RAW);

    gimbalFeedbackThread.start(NORMALPRIO - 1);
    gimbalThread.start(NORMALPRIO);

    chThdSleepMilliseconds(1000);
    LOG("Gimbal Yaw: %u, %f, Pitch: %u, %f",
        Gimbal::feedback[Gimbal::YAW].last_angle_raw, Gimbal::feedback[Gimbal::YAW].actual_angle,
        Gimbal::feedback[Gimbal::PITCH].last_angle_raw, Gimbal::feedback[Gimbal::PITCH].actual_angle);
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