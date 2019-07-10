//
// Created by Kerui Zhu on 7/9/2019.
//

#include "engineer_elevator_skd.h"

bool EngineerElevatorSKD::elevator_enabled;
bool EngineerElevatorSKD::elevator_locked;
bool EngineerElevatorSKD::aided_motor_enabled;
float EngineerElevatorSKD::target_height;
float EngineerElevatorSKD::target_velocity[2];
PIDController EngineerElevatorSKD::v2i_pid[4];
PIDController EngineerElevatorSKD::counter_balance_pid;


void EngineerElevatorSKD::init() {
    elevator_enable(false);
    elevator_lock(true);
    aided_motor_enable(false);
    change_pid_params(0, ELEVATOR_PID_V2I_PARAMS);
    change_pid_params(1, AIDED_MOTOR_PID_V2I_PARAMS);
    change_pid_params(2, ELEVATOR_PID_A2V_PARAMS);
}

void EngineerElevatorSKD::elevator_enable(bool enable) {
    elevator_enabled = enable;
}

void EngineerElevatorSKD::elevator_lock(bool locked) {
    elevator_locked = locked;
    if (elevator_locked){
        // If elevator is locked
        target_velocity[0] = 0;
    } else{
        // If elevator is unlocked
        target_velocity[0] = (target_height > EngineerElevatorIF::elevatorMotor[0].accmulate_angle)
                ? ENGINEER_ELEVATOR_VELOCITY : - ENGINEER_ELEVATOR_VELOCITY;
    }
}

void EngineerElevatorSKD::aided_motor_enable(bool enable) {
    aided_motor_enabled = enable;
}

void EngineerElevatorSKD::change_pid_params(int pid_id, PIDControllerBase::pid_params_t pid_params) {
    if (pid_id == 0){
        v2i_pid[0].change_parameters(pid_params);
        v2i_pid[1].change_parameters(pid_params);
        v2i_pid[0].clear_i_out();
        v2i_pid[1].clear_i_out();
    } else if (pid_id == 1){
        v2i_pid[2].change_parameters(pid_params);
        v2i_pid[3].change_parameters(pid_params);
        v2i_pid[2].clear_i_out();
        v2i_pid[3].clear_i_out();
    } else{
        counter_balance_pid.change_parameters(pid_params);
        counter_balance_pid.clear_i_out();
    }
}

void EngineerElevatorSKD::update_target_current() {
    if (elevator_enabled){
        EngineerElevatorIF::elevatorMotor[0].target_current = (uint16_t) v2i_pid[0].calc(EngineerElevatorIF::elevatorMotor[0].actual_velocity, target_velocity[0]);
        EngineerElevatorIF::elevatorMotor[1].target_current = (uint16_t) v2i_pid[1].calc(EngineerElevatorIF::elevatorMotor[1].actual_velocity, target_velocity[0]);

        EngineerElevatorIF::elevatorMotor[1].target_current += (uint16_t) counter_balance_pid.calc(EngineerElevatorIF::elevatorMotor[1].accmulate_angle, EngineerElevatorIF::elevatorMotor[0].accmulate_angle);
    }
    if (aided_motor_enabled){
        EngineerElevatorIF::aidedMotor[0].target_current = (uint16_t ) v2i_pid[2].calc(EngineerElevatorIF::aidedMotor[0].actual_velocity, target_velocity[1]);
        EngineerElevatorIF::aidedMotor[1].target_current = (uint16_t ) v2i_pid[3].calc(EngineerElevatorIF::aidedMotor[1].actual_velocity, target_velocity[1]);
    } else{
        EngineerElevatorIF::aidedMotor[0].target_current = EngineerElevatorIF::aidedMotor[1].target_current = 0;
    }
}

void EngineerElevatorSKD::EngineerElevatorThread::main() {
    setName("EngineerElevator");
    EngineerElevatorSKD::init();

    while (!shouldTerminate()){
        update_target_current();
        EngineerElevatorIF::send_currents();
        sleep(TIME_MS2I(100));
    }
}