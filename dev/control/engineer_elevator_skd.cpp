//
// Created by Kerui Zhu on 7/9/2019.
// Modified by LaiXinyi on 7/19/2019
//

#include "engineer_elevator_skd.h"

EngineerElevatorSKD::EngineerElevatorThread EngineerElevatorSKD::engineerElevatorThread;
float EngineerElevatorSKD::target_height;
float EngineerElevatorSKD::target_velocity[4];
bool EngineerElevatorSKD::elevator_enabled;
bool EngineerElevatorSKD::aided_motor_enabled;
PIDController EngineerElevatorSKD::v2i_pid[4];
PIDController EngineerElevatorSKD::a2v_pid[2];
PIDController EngineerElevatorSKD::counter_balance_pid;


void EngineerElevatorSKD::init() {
    target_velocity[0] = target_velocity[1] = target_velocity[2] = target_velocity[3] = 0;
    elevator_enable(false);
    aided_motor_enable(false);
    target_height = 0;
    change_pid_params(0, ELEVATOR_PID_V2I_PARAMS);
    change_pid_params(1, AIDED_MOTOR_PID_V2I_PARAMS);
    change_pid_params(2, ELEVATOR_PID_A2V_PARAMS);
    change_pid_params(3, {0,0,0,0,0});
}

void EngineerElevatorSKD::elevator_enable(bool enable) {
    if (elevator_enabled != enable) {
        // fixed the position at the time of switching
        set_target_height( EngineerElevatorIF::get_current_height() );
        // avoid setting repeatedly
        elevator_enabled = enable;
    }
}

void EngineerElevatorSKD::aided_motor_enable(bool enable) {
    if (aided_motor_enabled != enable) {
        // stop the motor at the time of switching
        set_aided_motor_velocity(0,0);
        // avoid setting repeatedly
        aided_motor_enabled = enable;
    }
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
    } else if (pid_id == 2){
        a2v_pid[0].change_parameters(pid_params);
        a2v_pid[1].change_parameters(pid_params);
        a2v_pid[0].clear_i_out();
        a2v_pid[1].clear_i_out();
    } else{
        counter_balance_pid.change_parameters(pid_params);
        counter_balance_pid.clear_i_out();
    }
}

void EngineerElevatorSKD::set_target_height(float new_height) {
    if (elevator_enabled)
        target_height = - new_height;   // target_height should take the negative value due to the direction
}

void EngineerElevatorSKD::set_aided_motor_velocity(float target_velocity_L, float target_velocity_R) {
    if (aided_motor_enabled) {
        target_velocity[2] = target_velocity_R;
        target_velocity[3] = target_velocity_L;
    }
}

void EngineerElevatorSKD::EngineerElevatorThread::main() {
    setName("EngineerElevator_SKD");
    EngineerElevatorSKD::init();

    while (!shouldTerminate()){

        // calculation for elevator
        float angle_0 = EngineerElevatorIF::elevatorMotor[0].present_angle;
        float angle_1 = EngineerElevatorIF::elevatorMotor[1].present_angle;
        target_velocity[0] = a2v_pid[0].calc(angle_0, target_height * ANGLE_HEIGHT_RATIO);
        target_velocity[1] = a2v_pid[1].calc(angle_0, target_height * ANGLE_HEIGHT_RATIO) + counter_balance_pid.calc(angle_1, angle_0);
        EngineerElevatorIF::elevatorMotor[0].target_current = (int16_t) v2i_pid[0].calc(EngineerElevatorIF::elevatorMotor[0].actual_velocity, target_velocity[0]);
        EngineerElevatorIF::elevatorMotor[1].target_current = (int16_t) v2i_pid[1].calc(EngineerElevatorIF::elevatorMotor[1].actual_velocity, target_velocity[1]);

        //calculation for aided motor
        EngineerElevatorIF::aidedMotor[0].target_current = (int16_t ) v2i_pid[2].calc(EngineerElevatorIF::aidedMotor[0].actual_velocity, target_velocity[2]);
        EngineerElevatorIF::aidedMotor[1].target_current = (int16_t ) v2i_pid[3].calc(EngineerElevatorIF::aidedMotor[1].actual_velocity, - target_velocity[3]);

        EngineerElevatorIF::send_currents();
        sleep(TIME_MS2I(2));
    }
}