//
// Created by Chen Qian on 10/29/21.
//

/**
 * @file can_motor_scheduler.cpp
 * @brief Class to drive can motors.
 *
 * @addtogroup CAN Driver
 * @{
 */

#include "can_motor_scheduler.h"

CANMotorSKD::feedbackThread CANMotorSKD::FeedbackThread;
CANMotorSKD::skdThread CANMotorSKD::SKDThread;
PIDController CANMotorSKD::v2iController[MOTOR_COUNT];
PIDController CANMotorSKD::a2vController[MOTOR_COUNT];
float *CANMotorSKD::feedbackV[MOTOR_COUNT];
float *CANMotorSKD::feedbackA[MOTOR_COUNT];

void CANMotorSKD::start(tprio_t SKD_PRIO, tprio_t FB_PRIO) {
    SKDThread.start(SKD_PRIO);
    FeedbackThread.start(FB_PRIO);
    for (int i = 0; i < MOTOR_COUNT; i++) {
        CANMotorSKD::feedbackV[i] = nullptr;
        CANMotorSKD::feedbackA[i] = nullptr;
    }
}

void CANMotorSKD::load_PID_params(motor_id_t id, bool is_a2v, PIDController::pid_params_t params) {
    if(id < 0 || id > MOTOR_COUNT) return; // Check validation
    if(is_a2v) {
        a2vController[id].change_parameters(params);
    } else {
        v2iController[id].change_parameters(params);
    }
}

void CANMotorSKD::switch_feedback_motor(motor_id_t id) {
    FeedbackThread.disp_id = id;
}

int  CANMotorSKD::get_torque_current(motor_id_t id){
    return CANMotorIF::motor_feedback[id].torque_current();
}

int  CANMotorSKD::get_PID_current(motor_id_t id) {
    return (int)SKDThread.PID_output[id];
}

void CANMotorSKD::set_target_angle(motor_id_t id, float target) {
    SKDThread.targetA[id] = target;
}

void CANMotorSKD::set_target_vel(motor_id_t id, float target) {
    SKDThread.targetV[id] = target;
}

void CANMotorSKD::set_target_current(motor_id_t id, int target) {
    SKDThread.output[id] = target;
}

void CANMotorSKD::register_custom_feedback(float *feedback_addr, CANMotorSKD::feedback_type_t fb_type,
                                           CANMotorCFG::motor_id_t motor_id) {
    switch (fb_type) {

        case angle:
            feedbackA[motor_id] = feedback_addr;
            break;
        case velocity:
            feedbackV[motor_id] = feedback_addr;
            break;
    }
}

void CANMotorSKD::unregister_custom_feedback(CANMotorSKD::feedback_type_t fb_type, CANMotorCFG::motor_id_t motor_id) {
    switch (fb_type) {
        case angle:
            feedbackA[motor_id] = nullptr;
            break;
        case velocity:
            feedbackV[motor_id] = nullptr;
            break;
    }
}

void CANMotorSKD::skdThread::main() {
    setName("HapticSKDThread");
    for(int i = 0; i < MOTOR_COUNT; i++) {
        v2iController[i].change_parameters(v2iParams[i]);
        a2vController[i].change_parameters(a2vParams[i]);
    }
    for (auto &i: targetA) {
        i = 0.0f;
    }
    for (auto &i: targetV) {
        i = 0.0f;
    }
    for (auto &i: output) {
        i = 0.0f;
    }
    while(!shouldTerminate()) {
        chSysLock();
        for (int i = 0; i < MOTOR_COUNT; i++) {
            if(enable_a2v[i]) {
                if(feedbackA[i] == nullptr) {
                    targetV[i] = a2vController[i].calc(CANMotorIF::motor_feedback[i].accumulate_angle(), targetA[i]);
                } else {
                    targetV[i] = a2vController[i].calc(*feedbackA[i], targetA[i]);
                }
            }
            if(enable_v2i[i]) {
                if(feedbackV[i] == nullptr) {
                    PID_output[i]=v2iController[i].calc(CANMotorIF::motor_feedback[i].actual_velocity, targetV[i]);
                } else {
                    PID_output[i]=v2iController[i].calc(*feedbackV[i], targetV[i]);
                }
                output[i] = (int)PID_output[i];
            } else {
                /// If disable the PID controller, clear the iterm so it does not bump.
                a2vController[i].clear_i_out();
                v2iController[i].clear_i_out();
            }
            CANMotorIF::set_current((motor_id_t)i, output[i]);
        }
        chSysUnlock();
        /// Might be the most efficient way...currently?
        if(CANMotorIF::enable_CAN_tx_frames[0][0]) {
            CANMotorIF::post_target_current(CANMotorBase::can_channel_1, 0x200);
        }
        if(CANMotorIF::enable_CAN_tx_frames[0][1]) {
            CANMotorIF::post_target_current(CANMotorBase::can_channel_1, 0x1FF);
        }
        if(CANMotorIF::enable_CAN_tx_frames[0][2]) {
            CANMotorIF::post_target_current(CANMotorBase::can_channel_1, 0x2FF);
        }
        if(CANMotorIF::enable_CAN_tx_frames[1][0]) {
            CANMotorIF::post_target_current(CANMotorBase::can_channel_2, 0x200);
        }
        if(CANMotorIF::enable_CAN_tx_frames[1][1]) {
            CANMotorIF::post_target_current(CANMotorBase::can_channel_2, 0x1FF);
        }
        if(CANMotorIF::enable_CAN_tx_frames[1][2]) {
            CANMotorIF::post_target_current(CANMotorBase::can_channel_2, 0x2FF);
        }
        sleep(TIME_MS2I(1));
    }
}

void CANMotorSKD::feedbackThread::main() {
    setName("feedback");
    while(!shouldTerminate()) {
        if(disp_id >= 0 && disp_id < MOTOR_COUNT) {
            /**TODO: re-enable Shell and display feedback*/
//            Shell::printf("!gy,%u,%.2f,%.2f,%.2f,%.2f,%d,%d" SHELL_NEWLINE_STR,
//                          SYSTIME,
//                          CANMotorIF::motor_feedback[disp_id].actual_angle, CANMotorSKD::SKDThread.targetA[disp_id],
//                          CANMotorIF::motor_feedback[disp_id].actual_velocity, CANMotorSKD::SKDThread.targetV[disp_id],
//                          CANMotorIF::motor_feedback[disp_id].torque_current(), (int)CANMotorSKD::SKDThread.PID_output[disp_id]);
        }
        sleep(TIME_MS2I(20));
    }
}

/** @} */