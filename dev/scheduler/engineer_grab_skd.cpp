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

void EngineerGrabSKD::set_belt_target_velocity(float targetVelocity_) {
//    EngineerGrabSKD::beltTargetVelocity = targetVelocity_;
    EngineerGrabSKD::target_velocity[BELT_L] = targetVelocity_;
    EngineerGrabSKD::target_velocity[BELT_R] = targetVelocity_;
}

void EngineerGrabSKD::invoke_rising() {
    if (armStatus == RAISED) return;
    armStatus = RISE_WAIT;
    target_angle[ROTATION_HAND] = RISE_TARGET;
}

void EngineerGrabSKD::invoke_lowering() {
    if (armStatus == HORIZONTALLED) return;
    armStatus = HORIZONTAL_WAIT;
    target_angle[ROTATION_HAND] = HORIZONTAL_TARGET;
}

void EngineerGrabSKD::invoke_relaxing() {
    if (armStatus == RELAXED) return;
    armStatus = RELAX_WAIT;
    target_angle[ROTATION_HAND] = RELAX_TARGET;
}

int EngineerGrabSKD::echo_status() {
    return (int) armStatus;
}

void EngineerGrabSKD::hold() {
    palWritePad(GPIOH, GPIOH_POWER1_CTRL, PAL_HIGH);
}

void EngineerGrabSKD::release() {
    palWritePad(GPIOH, GPIOH_POWER1_CTRL, PAL_LOW);
}

void EngineerGrabSKD::SKDThread::main() {
    while(!shouldTerminate()) {

        *EngGrabMechIF::target_current[BELT_L] = (int) v2iController[BELT_L].calc(EngGrabMechIF::feedback[BELT_L]->actual_velocity,
                                                                                  target_velocity[BELT_L] * (float) direction[BELT_L]);
        *EngGrabMechIF::target_current[BELT_R] = (int) v2iController[BELT_R].calc(EngGrabMechIF::feedback[BELT_R]->actual_velocity,
                                                                                  target_velocity[BELT_R] * (float) direction[BELT_R]);

        if (ABS_IN_RANGE(EngGrabMechIF::feedback[ROTATION_HAND]->actual_velocity, 20.0f)) {
            if (armStatus == HORIZONTAL_WAIT && ABS_IN_RANGE(EngGrabMechIF::feedback[ROTATION_HAND]->accumulated_angle() - HORIZONTAL_TARGET, ANGLE_MARGIN)) {
                armStatus = HORIZONTALLED;
            } else if (armStatus == RISE_WAIT && ABS_IN_RANGE(EngGrabMechIF::feedback[ROTATION_HAND]->accumulated_angle() - RISE_TARGET, ANGLE_MARGIN)) {
                armStatus = RAISED;
            } else if ((armStatus == RELAX_WAIT) && ABS_IN_RANGE(EngGrabMechIF::feedback[ROTATION_HAND]->accumulated_angle(), ANGLE_MARGIN)) {
                armStatus = RELAXED;
            }
        }

        if(armStatus == RELAXED) {
            *EngGrabMechIF::target_current[ROTATION_HAND] = (int) 0;
        } else {
            target_velocity[ROTATION_HAND] = a2vController[ROTATION_HAND].calc(EngGrabMechIF::feedback[ROTATION_HAND]->accumulated_angle(), target_angle[ROTATION_HAND]);
            *EngGrabMechIF::target_current[ROTATION_HAND] = (int) v2iController[ROTATION_HAND].calc(EngGrabMechIF::feedback[ROTATION_HAND]->actual_velocity, target_velocity[ROTATION_HAND]);
        }

        for (int i = GRABER_L; i < MOTOR_COUNT; i++) {
            target_angle[i] = EngGrabMechIF::feedback[ROTATION_HAND]->accumulated_angle() * (float) direction[i];
            target_velocity[i] = a2vController[i].calc(EngGrabMechIF::feedback[i]->accumulated_angle(), target_angle[i]);
            *EngGrabMechIF::target_current[i] = (int) v2iController[i].calc(EngGrabMechIF::feedback[i]->actual_velocity, target_velocity[i]);
        }

        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));
    }
}