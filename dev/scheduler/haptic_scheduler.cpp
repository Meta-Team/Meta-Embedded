//
// Created by 钱晨 on 10/29/21.
//

#include "haptic_scheduler.h"

haptic_scheduler::feedbackThread haptic_scheduler::FeedbackThread;
haptic_scheduler::skdThread haptic_scheduler::SKDThread;
PIDController haptic_scheduler::v2iController[CANBUS_MOTOR_CFG::MOTOR_COUNT];
PIDController haptic_scheduler::a2vController[CANBUS_MOTOR_CFG::MOTOR_COUNT];

void haptic_scheduler::start(tprio_t SKD_PRIO, tprio_t FB_PRIO) {
    SKDThread.start(SKD_PRIO);
    FeedbackThread.start(FB_PRIO);
}

void haptic_scheduler::load_PID_params(CANBUS_MOTOR_CFG::motor_id_t id, bool is_a2v, PIDController::pid_params_t params) {
    if(id < 0 || id > CANBUS_MOTOR_CFG::MOTOR_COUNT) return; // Check validation
    if(is_a2v) {
        a2vController[id].change_parameters(params);
    } else {
        v2iController[id].change_parameters(params);
    }
}

void haptic_scheduler::switch_feedback_motor(CANBUS_MOTOR_CFG::motor_id_t id) {
    FeedbackThread.disp_id = id;
}

int haptic_scheduler::get_torque_current(CANBUS_MOTOR_CFG::motor_id_t id){
    return can_motor_interface::motor_feedback[id].torque_current();
}

int haptic_scheduler::get_PID_current(CANBUS_MOTOR_CFG::motor_id_t id) {
    return SKDThread.PID_output[id];
}

void haptic_scheduler::set_target_angle(CANBUS_MOTOR_CFG::motor_id_t id, float target) {
    SKDThread.targetA[id] = target;
}

void haptic_scheduler::set_target_vel(CANBUS_MOTOR_CFG::motor_id_t id, float target) {
    SKDThread.targetV[id] = target;
}

void haptic_scheduler::set_target_current(CANBUS_MOTOR_CFG::motor_id_t id, int target) {
    SKDThread.output[id] = target;
}

void haptic_scheduler::skdThread::main() {
    setName("HapticSKDThread");
    for(int i = 0; i < CANBUS_MOTOR_CFG::MOTOR_COUNT; i++) {
        v2iController[i].change_parameters(CANBUS_MOTOR_CFG::v2iParams[i]);
        a2vController[i].change_parameters(CANBUS_MOTOR_CFG::a2vParams[i]);
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
        for (int i = 0; i < CANBUS_MOTOR_CFG::MOTOR_COUNT; i++) {
            if(CANBUS_MOTOR_CFG::enable_a2v[i]) {
                targetV[i] = a2vController[i].calc(can_motor_interface::motor_feedback[i].accumulate_angle(), targetA[i]);
            }
            if(CANBUS_MOTOR_CFG::enable_v2i[i]!=CANBUS_MOTOR_CFG::DISABLED) {
                PID_output[i]=v2iController[i].calc(can_motor_interface::motor_feedback[i].actual_velocity, targetV[i]);
                if (CANBUS_MOTOR_CFG::enable_v2i[i]==CANBUS_MOTOR_CFG::WORKING){
                    output[i] = (int)PID_output[i];
                }
            } else {
                /// If disable the PID controller, clear the iterm so it does not bump.
                a2vController[i].clear_i_out();
                v2iController[i].clear_i_out();
            }
            can_motor_interface::set_current((CANBUS_MOTOR_CFG::motor_id_t)i, output[i]);
        }
        can_motor_interface::post_target_current(CANMotorBase::can_channel_1, 0x1FF);
        sleep(TIME_MS2I(1));
    }
}

void haptic_scheduler::feedbackThread::main() {
    setName("feedback");
    while(!shouldTerminate()) {
        if(disp_id >= 0 && disp_id < CANBUS_MOTOR_CFG::MOTOR_COUNT) {
            Shell::printf("!gy,%u,%.2f,%.2f,%.2f,%.2f,%d,%d" SHELL_NEWLINE_STR,
                          SYSTIME,
                          can_motor_interface::motor_feedback[disp_id].actual_angle, haptic_scheduler::SKDThread.targetA[disp_id],
                          can_motor_interface::motor_feedback[disp_id].actual_velocity, haptic_scheduler::SKDThread.targetV[disp_id],
                          can_motor_interface::motor_feedback[disp_id].torque_current(), (int)haptic_scheduler::SKDThread.PID_output[disp_id]);
            sleep(TIME_MS2I(20));
        }
    }
}