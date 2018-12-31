//
// Created by liuzikai on 2018-12-30.
//

#include "gimbal_feedback_module.h"

void GimbalFeedbackModule::main() {

    setName("gimbal_feedback");

    while (!shouldTerminate()) {
        sprintf("!gyc" SHELL_NEWLINE_STR);
        sleep(TIME_MS2I(3000));
    }

}