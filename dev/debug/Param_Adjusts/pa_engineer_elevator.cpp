//
// Created by liuzikai on 2019-05-17.
//

/**
 * This unit test reuse unit for gimbal.
 */
#include "ch.hpp"
#include "hal.h"
#include "led.h"
#include "serial_shell.h"
#include "can_interface.h"
#include "common_macro.h"
#include "buzzer_scheduler.h"

#include "engineer_elevator_interface.h"
#include "engineer_elevator_skd.h"

using namespace chibios_rt;

unsigned const R = EngineerElevatorIF::R;
unsigned const L = EngineerElevatorIF::L;

unsigned const ELEVATOR_FEEDBACK_INTERVAL = 25; // [ms]

int const MAX_VELOCITY = {40960};  // absolute maximum, [qc/s]
int const MAX_CURRENT = 6000;  // [mA]

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);


class ElevatorFeedbackThread : public chibios_rt::BaseStaticThread<1024> {

public:

    bool enable_right_feedback = false;
    bool enable_left_feedback = false;

private:

    void main() final {

        setName("elevator_fb");

        while (!shouldTerminate()) {

            if (enable_right_feedback) {
                Shell::printf("!gy,%u,%.2f,%.2f,%.2f,%.2f,%d,%d" SHELL_NEWLINE_STR,
                              SYSTIME,
                              EngineerElevatorIF::elevatorMotor[R].present_angle, EngineerElevatorSKD::target_height * ANGLE_HEIGHT_RATIO,
                              EngineerElevatorIF::elevatorMotor[R].actual_velocity, EngineerElevatorSKD::target_velocity[0],
                              EngineerElevatorIF::elevatorMotor[R].actual_current, EngineerElevatorIF::elevatorMotor[R].target_current);
            }
            if (enable_left_feedback) {
                Shell::printf("!gp,%u,%.2f,%.2f,%.2f,%.2f,%d,%d" SHELL_NEWLINE_STR,
                              SYSTIME,
                              EngineerElevatorIF::elevatorMotor[L].present_angle, EngineerElevatorSKD::target_height * ANGLE_HEIGHT_RATIO,
                              EngineerElevatorIF::elevatorMotor[L].actual_velocity, EngineerElevatorSKD::target_velocity[1],
                              EngineerElevatorIF::elevatorMotor[L].actual_current, EngineerElevatorIF::elevatorMotor[L].target_current);
            }

            sleep(TIME_MS2I(ELEVATOR_FEEDBACK_INTERVAL));
        }
    }

} elevatorFeedbackThread;


static void cmd_elevator_enable(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "g_enable (0/1) NULL");
        return;
    }
    EngineerElevatorSKD::elevator_enable(*argv[0] - '0');
}

static void cmd_elevator_enable_fw(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "g_enable_fw DELETED");
        return;
    }
    // Do nothing
}

static void cmd_elevator_enable_feedback(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "g_enable_fb right(0/1) left(0/1)");
        return;
    }
    elevatorFeedbackThread.enable_right_feedback = *argv[0] - '0';
    elevatorFeedbackThread.enable_left_feedback = *argv[1] - '0';
}

static void cmd_elevator_fix_front_angle(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "g_fix");
        return;
    }
    for (unsigned i = 0; i < 2; i++)
        EngineerElevatorIF::elevatorMotor[i].clear_accumulate_angle();
}

void cmd_elevator_set_target_velocities(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "g_set_v target_velocity NULL");
        return;
    }
    PIDControllerBase::pid_params_t p = EngineerElevatorSKD::a2v_pid[0].get_parameters();
    EngineerElevatorSKD::change_pid_params(2,{p.kp, p.ki, p.kd, p.i_limit, Shell::atof(argv[0])});
}

static void cmd_elevator_set_target_height(BaseSequentialStream *chp, int argc, char **argv) {
    (void) argv;
    if (argc != 2) {
        shellUsage(chp, "g_set_angle target_angle NULL");
        return;
    }

    EngineerElevatorSKD::set_target_height(Shell::atof(argv[0]));
}

void cmd_elevator_set_parameters(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 7) {
        shellUsage(chp, "g_set_params yaw(0)/pitch(1) angle_to_v(0)/v_to_i(0) ki kp kd i_limit out_limit");
        chprintf(chp, "!pe" SHELL_NEWLINE_STR);  // echo parameters error
        return;
    }

    PIDControllerBase::pid_params_t p = {Shell::atof(argv[2]),
                                         Shell::atof(argv[3]),
                                         Shell::atof(argv[4]),
                                         Shell::atof(argv[5]),
                                         Shell::atof(argv[6])};

    if (*argv[1] == '0') EngineerElevatorSKD::change_pid_params(2, p);
    else EngineerElevatorSKD::change_pid_params(0, p);

    chprintf(chp, "!ps" SHELL_NEWLINE_STR); // echo parameters set
}

static inline void _cmd_elevator_echo_parameters(BaseSequentialStream *chp, PIDControllerBase::pid_params_t p) {
    chprintf(chp, "%f %f %f %f %f" SHELL_NEWLINE_STR, p.kp, p.ki, p.kd, p.i_limit, p.out_limit);
}

void cmd_elevator_echo_parameters(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "g_echo_params");
        return;
    }

    chprintf(chp, "angle_to_v:   ");
    _cmd_elevator_echo_parameters(chp, EngineerElevatorSKD::a2v_pid[0].get_parameters());
    chprintf(chp, "v_to_i:       ");
    _cmd_elevator_echo_parameters(chp, EngineerElevatorSKD::v2i_pid[0].get_parameters());
}

// Command lists for elevator controller test and adjustments
ShellCommand elevatorCotrollerCommands[] = {
        {"g_enable",      cmd_elevator_enable},
        {"g_enable_fb",   cmd_elevator_enable_feedback},
        {"g_fix",         cmd_elevator_fix_front_angle},
        {"g_set_v",       cmd_elevator_set_target_velocities},
        {"g_set_angle",   cmd_elevator_set_target_height},
        {"g_set_params",  cmd_elevator_set_parameters},
        {"g_echo_params", cmd_elevator_echo_parameters},
        {"g_enable_fw",   cmd_elevator_enable_fw},
        {nullptr,         nullptr}
};

int main(void) {

    halInit();
    System::init();
    LED::all_off();
    Shell::start(HIGHPRIO);
    Shell::addCommands(elevatorCotrollerCommands);

    can1.start(HIGHPRIO - 1);
    can2.start(HIGHPRIO - 2);
    chThdSleepMilliseconds(10);
    EngineerElevatorIF::init(&can2);

    elevatorFeedbackThread.start(NORMALPRIO - 1);
    EngineerElevatorSKD::engineerElevatorThread.start(NORMALPRIO);

    chThdSleepMilliseconds(500);
    BuzzerSKD::play_sound(BuzzerSKD::sound_startup_intel, LOWPRIO);

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