//
// Created by liuzikai on 2019-06-25.
// Edited by Qian Chen & Mo Kanya on 2019-07-05
//

#include "user.h"

User::UserThread User::userThread;
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

ShellCommand chassisCommands[] = {
        {"c_set_params", cmd_chassis_set_v2i_pid},
        {"c_set_target", cmd_chassis_set_target},
        {"c_echo_params", cmd_chassis_echo_params},
        {nullptr, nullptr}
};
void User::start(tprio_t prio) {
    userThread.start(prio);
}
void User::UserThread::main() {
    setName("User");
    while (!shouldTerminate()) {

        sleep(TIME_MS2I(USER_THREAD_INTERVAL));
    }
}