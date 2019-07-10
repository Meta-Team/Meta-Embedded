//
// Created by liuzikai on 2019-06-25.
// Edited by Qian Chen & Mo Kanya on 2019-07-05
//

#include "command.h"

User::UserThread User::userThread;
User::FeedbackThread User::feedbackThread;

time_msecs_t modified_test_end_time = 0;
time_msecs_t origin_test_end_time = 0;
time_msecs_t test_end_time = 0;

/** Shell Commands */
static void cmd_chassis_set_v2i_pid(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 5) {
        shellUsage(chp, "c_set_params ki kp kd i_limit out_limit");
        chprintf(chp, "!cpe" SHELL_NEWLINE_STR);
        return;
    }

    ChassisSKD::load_pid_params({0,0,0,0,0},
                                {Shell::atof(argv[0]),
                                 Shell::atof(argv[1]),
                                 Shell::atof(argv[2]),
                                 Shell::atof(argv[3]),
                                 Shell::atof(argv[4])});
}

static void cmd_chassis_set_target(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 4) {
        shellUsage(chp, "c_set_target vx(mm/s) vy(mm/s) w(deg/s, + for ccw) test_time(ms)");
        return;
    }

    ChassisSKD::set_target(Shell::atof(argv[0]), Shell::atof(argv[1]), Shell::atof(argv[2]));
    origin_test_end_time = TIME_I2MS(chVTGetSystemTime()) + (time_msecs_t) Shell::atoi(argv[3]);
    modified_test_end_time = TIME_I2MS(chVTGetSystemTime()) + (time_msecs_t) 4;

    if(origin_test_end_time > modified_test_end_time) {
        test_end_time = modified_test_end_time;
    } else {
        test_end_time = origin_test_end_time;
    }

}

static void cmd_chassis_echo_params(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "c_echo_params");
        return;
    }
    ChassisSKD::pid_params_t p = ChassisSKD::echo_pid_params();  // all PID params should be the same
    chprintf(chp, "Chassis PID: %f %f %f %f %f" SHELL_NEWLINE_STR, p.kp, p.ki, p.kd, p.i_limit, p.out_limit);
}

ShellCommand chassisCommands[] = {
        {"c_set_params", cmd_chassis_set_v2i_pid},
        {"c_set_target", cmd_chassis_set_target},
        {"c_echo_params", cmd_chassis_echo_params},
        {nullptr, nullptr}
};
void User::start(tprio_t user_prio, tprio_t feedback_prio) {
    userThread.start(user_prio);
    feedbackThread.start(feedback_prio);
}
void User::UserThread::main() {
    setName("User");
    Shell::addCommands(chassisCommands);
    while (!shouldTerminate()) {
        // Time check
        if (SYSTIME > test_end_time) {
            ChassisSKD::set_target(0, 0, 0);
            Shell::printf("!ce" SHELL_NEWLINE_STR);
        }
        sleep(TIME_MS2I(USER_THREAD_INTERVAL));
    }
}

void User::FeedbackThread::main() {
    setName("Feedback");
    while (!shouldTerminate()) {
        // feedback
        Shell::printf("!cv,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f" SHELL_NEWLINE_STR,
                      TIME_I2MS(chibios_rt::System::getTime()),
                      ChassisSKD::get_actual_velocity(ChassisBase::FR),
                      ChassisSKD::get_target_velocity(ChassisBase::FR),
                      ChassisSKD::get_actual_velocity(ChassisBase::FL),
                      ChassisSKD::get_target_velocity(ChassisBase::FL),
                      ChassisSKD::get_actual_velocity(ChassisBase::BL),
                      ChassisSKD::get_target_velocity(ChassisBase::BL),
                      ChassisSKD::get_actual_velocity(ChassisBase::BR),
                      ChassisSKD::get_target_velocity(ChassisBase::BR));
        sleep(TIME_MS2I(FEEDBACK_THREAD_INTERVAL));
    }
}