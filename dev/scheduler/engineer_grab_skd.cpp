//
// Created by Qian Chen on 3/27/21.
//

#include "engineer_grab_skd.h"
#include "math.h"

#define LOWER_TARGET                -110.0f
#define RISE_TARGET                 -150.0f
#define ANGLE_MARGIN                10.0f

PIDController                       engineerGrabSKD::v2iController[MOTOR_COUNT];
PIDController                       engineerGrabSKD::a2vController[MOTOR_COUNT];
engineerGrabSKD::SKDThread          engineerGrabSKD::skdThread;
engineerGrabSKD::install_direction  engineerGrabSKD::direction[MOTOR_COUNT];
float                               engineerGrabSKD::target_velocity[MOTOR_COUNT];
float                               engineerGrabSKD::target_angle[MOTOR_COUNT];
//float                               engineerGrabSKD::beltTargetVelocity;
//float                               engineerGrabSKD::grabberTargetVelocity[2];
engineerGrabSKD::arm_status_t       engineerGrabSKD::armStatus = LOWERED;

void engineerGrabSKD::start(tprio_t SKDThreadPRIO, engineerGrabSKD::install_direction direction_[5]) {
    skdThread.start(SKDThreadPRIO);
    for(int i = 0; i < MOTOR_COUNT; i++) {
        direction[i] = direction_[i];
    }
}

void engineerGrabSKD::load_v2i_pid_params(pid_params_t *pidParams) {
    for(int i = 0; i < MOTOR_COUNT; i++) {
        v2iController[i].change_parameters(pidParams[i]);
    }
}

void engineerGrabSKD::load_a2v_pid_params(pid_params_t *pidParams) {
    for(int i = 0; i < MOTOR_COUNT; i++) {
        a2vController[i].change_parameters(pidParams[i]);
    }
}

void engineerGrabSKD::set_belt_target_velocity(float targetVelocity_) {
//    engineerGrabSKD::beltTargetVelocity = targetVelocity_;
    engineerGrabSKD::target_velocity[BELT_L] = targetVelocity_;
    engineerGrabSKD::target_velocity[BELT_R] = targetVelocity_;
}

void engineerGrabSKD::invoke_rising() {
    if (armStatus != LOWERED) return;
    armStatus = RISING;
    target_angle[ROTATION_HAND] = RISE_TARGET;
}

void engineerGrabSKD::invoke_lowering() {
    if (armStatus != RISED) return;
    armStatus = LOWERING;
    target_angle[ROTATION_HAND] = LOWER_TARGET;
}

int engineerGrabSKD::echo_status() {
    return (int) armStatus;
}

void engineerGrabSKD::SKDThread::main() {
    while(!shouldTerminate()) {
        // For controlling the belt's motor
//        for (int i = 0; i < 2; i++) {
//            *EngGrabMechIF::target_current[i] = (int) v2iController[i].calc(EngGrabMechIF::feedback[i]->actual_velocity,
//                                                                            beltTargetVelocity * (float) direction[i]);
//        }

        // Self balancing of the 2006 for ore mine
//        for (int i = engineerGrabSKD::GRABER_L; i <= engineerGrabSKD::GRABER_R; i++) {
//            grabberTargetVelocity[i-3] = a2vController[i-3].calc(EngGrabMechIF::feedback[i]->accumulated_angle(),EngGrabMechIF::feedback[engineerGrabSKD::ROTATION_HAND]->accumulated_angle()* (float)direction[i]);
//            *EngGrabMechIF::target_current[i] = (int) v2iController[i-1].calc(EngGrabMechIF::feedback[i]->actual_velocity, grabberTargetVelocity[i-3]);
//        }

        *EngGrabMechIF::target_current[BELT_L] = (int) v2iController[BELT_L].calc(EngGrabMechIF::feedback[BELT_L]->actual_velocity,
                                                                                  target_velocity[BELT_L] * (float) direction[BELT_L]);
        *EngGrabMechIF::target_current[BELT_R] = (int) v2iController[BELT_R].calc(EngGrabMechIF::feedback[BELT_R]->actual_velocity,
                                                                                  target_velocity[BELT_R] * (float) direction[BELT_R]);

        if (ABS_IN_RANGE(EngGrabMechIF::feedback[ROTATION_HAND]->actual_velocity, 20.0f)) {
            if (armStatus == LOWERING && EngGrabMechIF::feedback[ROTATION_HAND]->accumulated_angle() > (LOWER_TARGET - ANGLE_MARGIN)) armStatus = LOWERED;
            else if (armStatus == RISING && EngGrabMechIF::feedback[ROTATION_HAND]->accumulated_angle() < (RISE_TARGET + ANGLE_MARGIN)) armStatus = RISED;
        }

        if (armStatus == LOWERED) {
            *EngGrabMechIF::target_current[ROTATION_HAND] = 0;
        } else {
            target_velocity[ROTATION_HAND] = a2vController[ROTATION_HAND].calc(EngGrabMechIF::feedback[ROTATION_HAND]->accumulated_angle(), target_angle[ROTATION_HAND]);
            *EngGrabMechIF::target_current[ROTATION_HAND] = (int) v2iController[ROTATION_HAND].calc(EngGrabMechIF::feedback[ROTATION_HAND]->actual_velocity, target_velocity[ROTATION_HAND]);
        }

        for (int i = GRABER_L; i < MOTOR_COUNT; i++) {
            target_angle[i] = EngGrabMechIF::feedback[ROTATION_HAND]->accumulated_angle() * (float) direction[i];
            target_velocity[i] = a2vController[i].calc(EngGrabMechIF::feedback[i]->accumulated_angle(), target_angle[i]);
            *EngGrabMechIF::target_current[i] = (int) v2iController[i].calc(EngGrabMechIF::feedback[i]->actual_velocity, target_velocity[i]);
        }
//        if(armStatus == RISING) {
//            if(EngGrabMechIF::feedback[EngGrabMechIF::ROTATION_HAND]->accumulated_angle() > -81.740516662f+3.0f) {
//                *EngGrabMechIF::target_current[EngGrabMechIF::ROTATION_HAND] = (int) -(3000.0f*cosf(0.5*(EngGrabMechIF::feedback[EngGrabMechIF::ROTATION_HAND]->accumulated_angle()+81.74f+90.0f)/180.0f*(float)M_PI));
//            } else {
//                *EngGrabMechIF::target_current[EngGrabMechIF::ROTATION_HAND] = (int)8000;
//            }
//        } else if (armStatus == LOWERING){
//            if(EngGrabMechIF::feedback[EngGrabMechIF::ROTATION_HAND]->accumulated_angle() < -81.740516662f-3.0f) {
//                *EngGrabMechIF::target_current[EngGrabMechIF::ROTATION_HAND] = (int) (5000.0f*cosf(0.5*(EngGrabMechIF::feedback[EngGrabMechIF::ROTATION_HAND]->accumulated_angle()+81.74f+80)/180.0f*(float)M_PI));
//            } else {
//                *EngGrabMechIF::target_current[EngGrabMechIF::ROTATION_HAND] = (int)-5000;
//            }
//        }

        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));
    }
}