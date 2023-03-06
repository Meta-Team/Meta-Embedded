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

#include "can_motor_controller.h"

CANMotorController::feedbackThread CANMotorController::FeedbackThread;
CANMotorController::skdThread CANMotorController::SKDThread;
PIDController CANMotorController::v2iController[MOTOR_COUNT];
PIDController CANMotorController::a2vController[MOTOR_COUNT];
float *CANMotorController::feedbackV[MOTOR_COUNT];
float *CANMotorController::feedbackA[MOTOR_COUNT];

void CANMotorController::start(tprio_t SKD_PRIO, tprio_t FB_PRIO, CANInterface *can1_, CANInterface *can2_) {
    CANMotorIF::init(can1_, can2_);
    SKDThread.start(SKD_PRIO);
    FeedbackThread.start(FB_PRIO);
    for (int i = 0; i < MOTOR_COUNT; i++) {
        CANMotorController::feedbackV[i] = nullptr;
        CANMotorController::feedbackA[i] = nullptr;
    }
}

void CANMotorController::load_PID_params(motor_id_t id, bool is_a2v, PIDController::pid_params_t params) {
    if(id < 0 || id >= MOTOR_COUNT) return; // Check validation
    if(is_a2v) {
        a2vController[id].change_parameters(params);
    } else {
        v2iController[id].change_parameters(params);
    }
}

void CANMotorController::shell_display(motor_id_t id, bool enable) {
    FeedbackThread.enable_feedback[id] = enable;
}

int  CANMotorController::get_torque_current(motor_id_t id){
    return CANMotorIF::motor_feedback[id].torque_current();
}

int  CANMotorController::get_PID_current(motor_id_t id) {
    return (int)SKDThread.PID_output[id];
}

void CANMotorController::set_target_angle(motor_id_t id, float target) {
    SKDThread.targetA[id] = target;
}

void CANMotorController::set_target_vel(motor_id_t id, float target) {
    SKDThread.targetV[id] = target;
}

void CANMotorController::set_target_current(motor_id_t id, int target) {
    SKDThread.output[id] = target;
}

void CANMotorController::register_custom_feedback(float *feedback_addr, CANMotorController::feedback_type_t fb_type,
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

void CANMotorController::unregister_custom_feedback(CANMotorController::feedback_type_t fb_type, CANMotorCFG::motor_id_t motor_id) {
    switch (fb_type) {
        case angle:
            feedbackA[motor_id] = nullptr;
            break;
        case velocity:
            feedbackV[motor_id] = nullptr;
            break;
    }
}

void CANMotorController::skdThread::main() {
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
        /// Multi thread should take no effects on the timing as sending will cause chSysLock
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
        // This command will adjust the sleep time to ensure controller was triggered at a constant rate.
        // Sampling time could be critical when designing controller. Non-uniform sampling time would cause robustness problem in control system.
        tprio_t PRIO = this->getPriorityX();
        unsigned long sleep_time = THREAD_INTERVAL - (TIME_I2US(chVTGetSystemTimeX()) + PRIO)%THREAD_INTERVAL;
        sleep(TIME_US2I(sleep_time));
    }
}

void CANMotorController::feedbackThread::main() {
    setName("feedback");
    while(!shouldTerminate()) {
        for (int i = 0; i < CANMotorCFG::MOTOR_COUNT; i++) {
            if(enable_feedback[i]) {
            Shell::printf("!fb,%u,%u,%.2f,%.2f,%.2f,%.2f,%d,%d" SHELL_NEWLINE_STR,
                          SYSTIME,
                          i, // Motor ID
                          CANMotorIF::motor_feedback[i].actual_angle, CANMotorController::SKDThread.targetA[i],
                          CANMotorIF::motor_feedback[i].actual_velocity, CANMotorController::SKDThread.targetV[i],
                          CANMotorIF::motor_feedback[i].torque_current(), (int)CANMotorController::SKDThread.PID_output[i]);
            }
        }
        tprio_t PRIO = this->getPriorityX();
        unsigned long sleep_time = THREAD_INTERVAL - (TIME_I2US(chVTGetSystemTimeX()) + PRIO)%THREAD_INTERVAL;
        sleep(TIME_US2I(sleep_time));
    }
}

/** @} */