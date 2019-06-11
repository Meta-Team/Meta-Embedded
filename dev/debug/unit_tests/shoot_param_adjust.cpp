//
// Created by liuzikai on 2019-06-11.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "shell.h"

#include "can_interface.h"

#include "shoot.h"

#include "remote_interpreter.h"

using namespace chibios_rt;

// Duplicate of motor_id_t in GimbalInterface to reduce code
const unsigned BULLET = Shoot::BULLET;

const char MOTOR_CHAR = 'y';

// Calculation interval for shoot thread
const unsigned SHOOT_THREAD_INTERVAL = 2;    // [ms]
const unsigned SHOOT_FEEDBACK_INTERVAL = 25; // [ms]

const float MAX_VELOCITY = 720;  // absolute maximum, [degree/s]
const int MAX_CURRENT = 5000;    // [mA]

bool motor_enabled = false;

float target_v = 0;

CANInterface can1(&CAND1);

class ShootFeedbackThread : public chibios_rt::BaseStaticThread<1024> {

public:

    bool enable_bullet_feedback = false;

private:

    void main() final {

        setName("shoot_fb");

        while (!shouldTerminate()) {

            if (enable_bullet_feedback) {
                Shell::printf("!gy,%u,%.2f,%.2f,%.2f,%.2f,%d,%d" SHELL_NEWLINE_STR,
                              SYSTIME,
                              0.0f, 0.0f,
                              Shoot::feedback[BULLET].actual_velocity, target_v,
                              Shoot::feedback[BULLET].actual_current, Shoot::target_current[BULLET]);
            }

            sleep(TIME_MS2I(SHOOT_FEEDBACK_INTERVAL));
        }
    }

} shootFeedbackThread;


/**
 * @brief set enabled states of bullet motors
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_shoot_enable(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2 || (*argv[0] != '0' && *argv[0] != '1') || (*argv[1] != '0' && *argv[1] != '1')) {
        shellUsage(chp, "g_enable bullet(0/1) NULL(0/1)");
        return;
    }
    motor_enabled = *argv[0] - '0';
}

/**
 * @brief set enabled state of friction wheels
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_shoot_enable_fw(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1 || (*argv[0] != '0' && *argv[0] != '1')) {
        shellUsage(chp, "g_enable_fw 0/1");
        return;
    }
    if (*argv[0] == '1') {
        Shoot::set_friction_wheels(0.1);
    } else {
        Shoot::set_friction_wheels(0);
    }
    GimbalInterface::send_gimbal_currents();
}

/**
 * @brief set feedback enable states
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_shoot_enable_feedback(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2 || (*argv[0] != '0' && *argv[0] != '1') || (*argv[1] != '0' && *argv[1] != '1')) {
        shellUsage(chp, "g_enable_fb bullet(0/1) NULL(0/1)");
        return;
    }
    shootFeedbackThread.enable_bullet_feedback = *argv[0] - '0';
}


/**
 * @brief set front_angle_raw with current actual angle
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_shoot_fix_front_angle(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "g_fix");
        return;
    }
    Shoot::feedback[BULLET].reset_front_angle();

//    chprintf(chp, "!f" SHELL_NEWLINE_STR);
}

void _cmd_shoot_clear_i_out() {
    for (int i = 0; i < 2; i++) {
        Shoot::v2i_pid[i].clear_i_out();
    }
}

/**
 * @brief set target velocity of bullet
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_shoot_set_target_velocities(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "g_set_v bullet NULL");
        return;
    }

    target_v = Shell::atof(argv[0]);
    _cmd_shoot_clear_i_out();
}

/**
 * @brief set target angle of yaw and pitch and enable pos_to_v_pid
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_shoot_set_target_angle(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "g_set_angle NULL NULL");
        return;
    }

    // Do nothing
}

/**
 * @brief set pid parameters
 * @param chp
 * @param argc
 * @param argv
 */
void cmd_shoot_set_parameters(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 7) {
        shellUsage(chp, "g_set_params bullet(0)/NULL(1) NULL(0)/v_to_i(1) ki kp kd i_limit out_limit");
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }

    Shoot::pid_params_t bullet_v2i_params = Shoot::v2i_pid[0].get_parameters();

    Shoot::pid_params_t *p = nullptr;
    if (*argv[0] == '0' && *argv[1] == '1') p = &bullet_v2i_params;
    else {
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }

    *p = {Shell::atof(argv[2]),
          Shell::atof(argv[3]),
          Shell::atof(argv[4]),
          Shell::atof(argv[5]),
          Shell::atof(argv[6])};

    Shoot::change_pid_params(bullet_v2i_params);

    chprintf(chp, "!ps" SHELL_NEWLINE_STR); // echo parameters set
}

/**
 * @brief helper function for cmd_shoot_echo_parameters()
 */
static inline void _cmd_shoot_echo_parameters(BaseSequentialStream *chp, Shoot::pid_params_t p) {
    chprintf(chp, "%f %f %f %f %f" SHELL_NEWLINE_STR, p.kp, p.ki, p.kd, p.i_limit, p.out_limit);
}

/**
 * @brief echo pid parameters
 * @param chp
 * @param argc
 * @param argv
 */
void cmd_shoot_echo_parameters(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "g_echo_params");
        return;
    }

    chprintf(chp, "bullet v_to_i:       ");
    _cmd_shoot_echo_parameters(chp, Shoot::v2i_pid[BULLET].get_parameters());
}

// Command lists for shoot controller test and adjustments
ShellCommand shootCotrollerCommands[] = {
        {"g_enable",      cmd_shoot_enable},
        {"g_enable_fb",   cmd_shoot_enable_feedback},
        {"g_fix",         cmd_shoot_fix_front_angle},
        {"g_set_v",       cmd_shoot_set_target_velocities},
        {"g_set_angle",   cmd_shoot_set_target_angle},
        {"g_set_params",  cmd_shoot_set_parameters},
        {"g_echo_params", cmd_shoot_echo_parameters},
        {"g_enable_fw",   cmd_shoot_enable_fw},
        {nullptr,         nullptr}
};


class ShootDebugThread : public BaseStaticThread<1024> {
protected:
    void main() final {

        setName("shoot");

        // TODO: load default parameters here
        Shoot::change_pid_params({20.0f, 0.0f, 0.0f, 1000.0f, 3000.0f});

        while (!shouldTerminate()) {

            if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) {
                // Calculation and check
                if (motor_enabled) {

                    // Perform velocity check
                    if (Shoot::feedback[BULLET].actual_velocity > MAX_VELOCITY) {
                        Shell::printf("!d%cv" SHELL_NEWLINE_STR, MOTOR_CHAR);
                        motor_enabled = false;
                        continue;
                    }

                    // Calculate from velocity to current
                    Shoot::calc_motor_(Shoot::BULLET, Shoot::feedback[BULLET].actual_velocity, target_v);

                    // Perform current check
                    if (Shoot::target_current[BULLET] > MAX_CURRENT || Shoot::target_current[BULLET] < -MAX_CURRENT) {
                        Shell::printf("!d%cc" SHELL_NEWLINE_STR, MOTOR_CHAR);
                        motor_enabled = false;
                        continue;
                    }

                }

                // This operation should be after calculation since motor can get disabled if check failed
                // This operation should always perform, instead of being put in a 'else' block
                if (!motor_enabled) Shoot::target_current[BULLET] = 0;

                // Do nothing to friction wheels, controlled by command

            } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE) {
                // TODO: revise shoot speed and friction wheel speed by Remote here
                Shoot::calc(Remote::rc.ch3 * 10);
                Shoot::set_friction_wheels(0.2);
            } else {
                Shoot::target_current[BULLET] = 0;
                Shoot::set_friction_wheels(0);
            }


            // Send currents
            GimbalInterface::send_gimbal_currents();

            sleep(TIME_MS2I(SHOOT_THREAD_INTERVAL));
        }
    }
} shootThread;


int main(void) {

    halInit();
    System::init();
    LED::all_off();
    Shell::start(HIGHPRIO);
    Shell::addCommands(shootCotrollerCommands);

    can1.start(HIGHPRIO - 1);
    chThdSleepMilliseconds(10);
    GimbalInterface::init(&can1, 0, 0);

    Remote::start_receive();

    shootFeedbackThread.start(NORMALPRIO - 1);
    shootThread.start(NORMALPRIO);

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