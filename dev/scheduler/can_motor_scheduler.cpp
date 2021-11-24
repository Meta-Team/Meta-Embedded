//
// Created by Chen Qian on 10/29/21.
//

#include "can_motor_scheduler.h"

CANMotorSKD::feedbackThread CANMotorSKD::FeedbackThread;
CANMotorSKD::skdThread CANMotorSKD::SKDThread;
PIDController CANMotorSKD::v2iController[CANMotorCFG::MOTOR_COUNT];
PIDController CANMotorSKD::a2vController[CANMotorCFG::MOTOR_COUNT];

void CANMotorSKD::start(tprio_t SKD_PRIO, tprio_t FB_PRIO) {
    SKDThread.start(SKD_PRIO);
    FeedbackThread.start(FB_PRIO);
}

void CANMotorSKD::load_PID_params(CANMotorCFG::motor_id_t id, bool is_a2v, PIDController::pid_params_t params) {
    if(id < 0 || id > CANMotorCFG::MOTOR_COUNT) return; // Check validation
    if(is_a2v) {
        a2vController[id].change_parameters(params);
    } else {
        v2iController[id].change_parameters(params);
    }
}

void CANMotorSKD::switch_feedback_motor(CANMotorCFG::motor_id_t id) {
    FeedbackThread.disp_id = id;
}

int  CANMotorSKD::get_torque_current(CANMotorCFG::motor_id_t id){
    return CANMotorInterface::motor_feedback[id].torque_current();
}

int  CANMotorSKD::get_PID_current(CANMotorCFG::motor_id_t id) {
    return SKDThread.PID_output[id];
}

void CANMotorSKD::set_target_angle(CANMotorCFG::motor_id_t id, float target) {
    SKDThread.targetA[id] = target;
}

void CANMotorSKD::set_target_vel(CANMotorCFG::motor_id_t id, float target) {
    SKDThread.targetV[id] = target;
}

void CANMotorSKD::set_target_current(CANMotorCFG::motor_id_t id, int target) {
    SKDThread.output[id] = target;
}

void CANMotorSKD::skdThread::main() {
    setName("HapticSKDThread");
    for(int i = 0; i < CANMotorCFG::MOTOR_COUNT; i++) {
        v2iController[i].change_parameters(CANMotorCFG::v2iParams[i]);
        a2vController[i].change_parameters(CANMotorCFG::a2vParams[i]);
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
        for (int i = 0; i < CANMotorCFG::MOTOR_COUNT; i++) {
            if(CANMotorCFG::enable_a2v[i]) {
                targetV[i] = a2vController[i].calc(CANMotorInterface::motor_feedback[i].accumulate_angle(), targetA[i]);
            }
            if(CANMotorCFG::enable_v2i[i]) {
                PID_output[i]=v2iController[i].calc(CANMotorInterface::motor_feedback[i].actual_velocity, targetV[i]);
                output[i] = (int)PID_output[i];
            } else {
                /// If disable the PID controller, clear the iterm so it does not bump.
                a2vController[i].clear_i_out();
                v2iController[i].clear_i_out();
            }
            CANMotorInterface::set_current((CANMotorCFG::motor_id_t)i, output[i]);
        }
        chSysUnlock();
        /// Might be the most efficient way...currently?
        if(CANMotorInterface::enable_CAN_tx_frames[0][0]) {
            CANMotorInterface::post_target_current(CANMotorBase::can_channel_1, 0x200);
        }
        if(CANMotorInterface::enable_CAN_tx_frames[0][1]) {
            CANMotorInterface::post_target_current(CANMotorBase::can_channel_1, 0x1FF);
        }
        if(CANMotorInterface::enable_CAN_tx_frames[0][2]) {
            CANMotorInterface::post_target_current(CANMotorBase::can_channel_1, 0x2FF);
        }
        if(CANMotorInterface::enable_CAN_tx_frames[1][0]) {
            CANMotorInterface::post_target_current(CANMotorBase::can_channel_2, 0x200);
        }
        if(CANMotorInterface::enable_CAN_tx_frames[1][1]) {
            CANMotorInterface::post_target_current(CANMotorBase::can_channel_2, 0x1FF);
        }
        if(CANMotorInterface::enable_CAN_tx_frames[1][2]) {
            CANMotorInterface::post_target_current(CANMotorBase::can_channel_2, 0x2FF);
        }
        sleep(TIME_MS2I(1));
    }
}

void CANMotorSKD::feedbackThread::main() {
    setName("feedback");
    while(!shouldTerminate()) {
        if(disp_id >= 0 && disp_id < CANMotorCFG::MOTOR_COUNT) {
            Shell::printf("!gy,%u,%.2f,%.2f,%.2f,%.2f,%d,%d" SHELL_NEWLINE_STR,
                          SYSTIME,
                          CANMotorInterface::motor_feedback[disp_id].actual_angle, CANMotorSKD::SKDThread.targetA[disp_id],
                          CANMotorInterface::motor_feedback[disp_id].actual_velocity, CANMotorSKD::SKDThread.targetV[disp_id],
                          CANMotorInterface::motor_feedback[disp_id].torque_current(), (int)CANMotorSKD::SKDThread.PID_output[disp_id]);
            sleep(TIME_MS2I(20));
        }
    }
}