//
// Created by Qian Chen on 3/27/21.
//

#include "engineer_grab_skd.h"
#include "math.h"

#define HORIZONTAL_TARGET           -90.0f
#define RISE_TARGET                 -155.73f
#define RELAX_TARGET                0.0f
#define ANGLE_MARGIN                10.0f

PIDController                       EngineerGrabSKD::v2iController[MOTOR_COUNT];
PIDController                       EngineerGrabSKD::a2vController[MOTOR_COUNT];
EngineerGrabSKD::SKDThread          EngineerGrabSKD::skdThread;
EngineerGrabSKD::install_direction  EngineerGrabSKD::direction[MOTOR_COUNT];
float                               EngineerGrabSKD::target_velocity[MOTOR_COUNT];
float                               EngineerGrabSKD::target_angle[MOTOR_COUNT];
//float                               EngineerGrabSKD::beltTargetVelocity;
//float                               EngineerGrabSKD::grabberTargetVelocity[2];
EngineerGrabSKD::arm_status_t       EngineerGrabSKD::armStatus = HORIZONTALLED;

void EngineerGrabSKD::start(tprio_t SKDThreadPRIO, EngineerGrabSKD::install_direction direction_[5]) {
    skdThread.start(SKDThreadPRIO);
    for(int i = 0; i < MOTOR_COUNT; i++) {
        direction[i] = direction_[i];
    }
    palSetPad(GPIOH, GPIOH_POWER1_CTRL);
}

void EngineerGrabSKD::load_v2i_pid_params(pid_params_t *pidParams) {
    for(int i = 0; i < MOTOR_COUNT; i++) {
        v2iController[i].change_parameters(pidParams[i]);
    }
}

void EngineerGrabSKD::load_a2v_pid_params(pid_params_t *pidParams) {
    for(int i = 0; i < MOTOR_COUNT; i++) {
        a2vController[i].change_parameters(pidParams[i]);
    }
}

void EngineerGrabSKD::set_angle(float target_angle_) {
    target_angle[ROTATION_HAND] = target_angle_;
}

void EngineerGrabSKD::hold() {
    palWritePad(GPIOH, GPIOH_POWER1_CTRL, PAL_HIGH);
}

void EngineerGrabSKD::release() {
    if(ABS_IN_RANGE(EngGrabMechIF::feedback[ROTATION_HAND]->accumulated_angle(), 75.0f)) return;
    palWritePad(GPIOH, GPIOH_POWER1_CTRL, PAL_LOW);
}

void EngineerGrabSKD::SKDThread::main() {
    while(!shouldTerminate()) {

        for (int i = GRABER_L; i < MOTOR_COUNT; i++) {
            target_angle[i] = EngGrabMechIF::feedback[ROTATION_HAND]->accumulated_angle() * (float) direction[i];
            target_velocity[i] = a2vController[i].calc(EngGrabMechIF::feedback[i]->accumulated_angle(), target_angle[i]);
            *EngGrabMechIF::target_current[i] = (int) v2iController[i].calc(EngGrabMechIF::feedback[i]->actual_velocity, target_velocity[i]);
        }

        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));
    }
}