//
// Created by Kerui Zhu on 7/9/2019.
// Modified by LaiXinyi on 7/19/2019
//

#include "engineer_elevator_skd.h"

float EngineerElevatorSKD::target_height = 0;
float EngineerElevatorSKD::target_velocity[4] = {0, 0, 0, 0};
bool EngineerElevatorSKD::elevator_enabled = false;
bool EngineerElevatorSKD::aided_motor_enabled = false;
PIDController EngineerElevatorSKD::v2i_pid[4];
PIDController EngineerElevatorSKD::a2v_pid[2];
PIDController EngineerElevatorSKD::counter_balance_pid;

EngineerElevatorSKD::SKDThread EngineerElevatorSKD::skdThread;


void EngineerElevatorSKD::start(tprio_t thread_prio) {
    skdThread.start(thread_prio);
}

void EngineerElevatorSKD::elevator_enable(bool enable) {
    elevator_enabled = enable;
}

void EngineerElevatorSKD::aided_motor_enable(bool enable) {
    aided_motor_enabled = enable;
}

void EngineerElevatorSKD::load_pid_params(pid_id_t pid_id, PIDControllerBase::pid_params_t pid_params) {
    switch (pid_id){
        case ELEVATOR_A2V:
            a2v_pid[0].change_parameters(pid_params);
            a2v_pid[1].change_parameters(pid_params);
            a2v_pid[0].clear_i_out();
            a2v_pid[1].clear_i_out();
            break;
        case ELEVATOR_V2I:
            v2i_pid[0].change_parameters(pid_params);
            v2i_pid[1].change_parameters(pid_params);
            v2i_pid[0].clear_i_out();
            v2i_pid[1].clear_i_out();
            break;
        case AIDED_WHEEL_V2I:
            v2i_pid[2].change_parameters(pid_params);
            v2i_pid[3].change_parameters(pid_params);
            v2i_pid[2].clear_i_out();
            v2i_pid[3].clear_i_out();
            break;
        case BALANCE_PID:
            counter_balance_pid.change_parameters(pid_params);
            counter_balance_pid.clear_i_out();
        default:
            break;
    }
}

void EngineerElevatorSKD::set_target_height(float new_height) {
    target_height = - new_height;
}

void EngineerElevatorSKD::set_aided_motor_velocity(float target_velocity_) {
    target_velocity[2] = target_velocity[3] = - target_velocity_;
}

float EngineerElevatorSKD::get_current_height() {
    return EngineerElevatorIF::get_current_height();
}

void EngineerElevatorSKD::SKDThread::main() {
    setName("ElevatorSKD");

    while (!shouldTerminate()){

        /// Elevator
        if (elevator_enabled) {
            float angle_0 = EngineerElevatorIF::elevatorMotor[0].present_angle;
            float angle_1 = EngineerElevatorIF::elevatorMotor[1].present_angle;
            target_velocity[0] = a2v_pid[0].calc(angle_0, target_height * ANGLE_HEIGHT_RATIO);
            target_velocity[1] = a2v_pid[1].calc(angle_0, target_height * ANGLE_HEIGHT_RATIO) +
                                 counter_balance_pid.calc(angle_1, angle_0);
            EngineerElevatorIF::elevatorMotor[0].target_current = (int16_t) v2i_pid[0].calc(
                    EngineerElevatorIF::elevatorMotor[0].actual_velocity, target_velocity[0]);
            EngineerElevatorIF::elevatorMotor[1].target_current = (int16_t) v2i_pid[1].calc(
                    EngineerElevatorIF::elevatorMotor[1].actual_velocity, target_velocity[1]);
        } else{
            EngineerElevatorIF::elevatorMotor[0].target_current = EngineerElevatorIF::elevatorMotor[1].target_current = 0;
        }

        /// Aided motor
        if (aided_motor_enabled) {
            EngineerElevatorIF::aidedMotor[0].target_current = (int16_t ) v2i_pid[2].calc(EngineerElevatorIF::aidedMotor[0].actual_velocity, target_velocity[2]);
            EngineerElevatorIF::aidedMotor[1].target_current = (int16_t ) v2i_pid[3].calc(EngineerElevatorIF::aidedMotor[1].actual_velocity, - target_velocity[3]);
        } else {
            EngineerElevatorIF::aidedMotor[0].target_current = EngineerElevatorIF::aidedMotor[1].target_current = 0;
        }

        /// Apply changes
        EngineerElevatorIF::send_currents();

        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));
    }
}