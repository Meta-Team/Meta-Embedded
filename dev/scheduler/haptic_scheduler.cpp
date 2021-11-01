//
// Created by 钱晨 on 10/29/21.
//

#include "haptic_scheduler.h"

haptic_scheduler::feedbackThread haptic_scheduler::FeedbackThread;
haptic_scheduler::skdThread haptic_scheduler::SKDThread;
PIDController haptic_scheduler::v2iController[can_motor_interface::MOTOR_COUNT];
PIDController haptic_scheduler::a2vController[can_motor_interface::MOTOR_COUNT];

void haptic_scheduler::start(tprio_t SKD_PRIO, tprio_t FB_PRIO) {
    SKDThread.start(SKD_PRIO);
    FeedbackThread.start(FB_PRIO);
}

void haptic_scheduler::load_PID_params(can_motor_interface::motor_id_t id, bool is_a2v, PIDController::pid_params_t params) {
    if(id < 0 || id > can_motor_interface::MOTOR_COUNT) return; // Check validation
    if(is_a2v) {
        a2vController[id].change_parameters(params);
    } else {
        v2iController[id].change_parameters(params);
    }
}

void haptic_scheduler::switch_feedback_motor(can_motor_interface::motor_id_t id) {
    FeedbackThread.disp_id = id;
}

void haptic_scheduler::skdThread::main() {
    setName("HapticSKDThread");
    for(int i = 0; i < can_motor_interface::MOTOR_COUNT; i++) {
        v2iController[i].change_parameters(can_motor_interface::v2iParams[i]);
        a2vController[i].change_parameters(can_motor_interface::a2vParams[i]);
    }
    for (auto &i: target) {
        i = 0.0f;
    }
    for (auto &i: targetV) {
        i = 0.0f;
    }
    for (auto &i: output) {
        i = 0.0f;
    }
    while(!shouldTerminate()) {
        for (int i = 0; i < can_motor_interface::MOTOR_COUNT; i++) {
            targetV[i] = a2vController[i].calc(can_motor_interface::motor_feedback[i].accumulate_angle(), target[i]);
            output[i] = (int) v2iController[i].calc(can_motor_interface::motor_feedback[i].actual_velocity, targetV[i]);
            can_motor_interface::set_current((can_motor_interface::motor_id_t)i, output[i]);
        }
        can_motor_interface::post_target_current(CANMotorBase::can_channel_1, 0x1FF);
        sleep(TIME_MS2I(1));
    }
}

void haptic_scheduler::feedbackThread::main() {
    setName("feedback");
    while(!shouldTerminate()) {
        Shell::printf("!gy,%u,%.2f,%.2f,%.2f,%.2f,%d,%d" SHELL_NEWLINE_STR,
                      SYSTIME,
                      can_motor_interface::motor_feedback[disp_id].actual_angle, haptic_scheduler::SKDThread.target[disp_id],
                      can_motor_interface::motor_feedback[disp_id].actual_velocity, haptic_scheduler::SKDThread.targetV[disp_id],
                      (int)(can_motor_interface::motor_feedback[disp_id].torque()*1000.0f), (int)(can_motor_interface::motor_feedback[disp_id].torque()*1000.0f));
        sleep(TIME_MS2I(20));
    }
}