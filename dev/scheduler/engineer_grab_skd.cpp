//
// Created by Qian Chen on 3/27/21.
//

#include "engineer_grab_skd.h"

PIDController                       engineerGrabSKD::v2iController[2];
engineerGrabSKD::SKDThread          engineerGrabSKD::skdThread;
engineerGrabSKD::install_direction  engineerGrabSKD::direction[2];
float                               engineerGrabSKD::targetVelocity;

void engineerGrabSKD::start(tprio_t SKDThreadPRIO, engineerGrabSKD::install_direction direction_[2]) {
    skdThread.start(SKDThreadPRIO);
    for(int i = 0; i < 2; i++) {
        direction[i] = direction_[i];
    }
}

void engineerGrabSKD::load_pid_params(PIDController::pid_params_t pidParams[2]) {
    for(int i = 0; i < 2; i++) {
        v2iController[i].change_parameters(pidParams[i]);
    }
}

void engineerGrabSKD::set_target_velocity(float targetVelocity_) {
    engineerGrabSKD::targetVelocity = targetVelocity_;
}

void engineerGrabSKD::SKDThread::main() {
    while(!shouldTerminate()) {
        // For controlling the belt's motor
        for (int i = 0; i < 2; i++) {
            *EngGrabMechIF::target_current[i] = (int) v2iController[i].calc(EngGrabMechIF::feedback[i]->actual_velocity,
                                                                      targetVelocity* (float) direction[i]);
        }
        // For controlling the grab motor
        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));
    }
}