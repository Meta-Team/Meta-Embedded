//
// Created by Kerui Zhu on 7/9/2019.
//

#include "engineer_chassis_skd.h"

EngineerChassisSKD::EngineerChassisThread EngineerChassisSKD::engineerChassisThread;
float EngineerChassisSKD::target_velocity[ENGINEER_CHASSIS_MOTOR_COUNT];
bool EngineerChassisSKD::enable;
float EngineerChassisSKD::target_vx;
float EngineerChassisSKD::target_vy;
float EngineerChassisSKD::target_w;
PIDController EngineerChassisSKD::pid[ENGINEER_CHASSIS_MOTOR_COUNT];

void EngineerChassisSKD::init(){
    lock();
    change_pid_params(CHASSIS_PID_V2I_PARAMS);
}

void EngineerChassisSKD::lock() {
    enable = false;
    target_vx = target_vy = target_w = 0;
}

void EngineerChassisSKD::unlock() {
    enable = true;
    for (PIDController pidController : pid) pidController.clear_i_out();
}

void EngineerChassisSKD::change_pid_params(PIDControllerBase::pid_params_t pid_params) {
    for (int i = 0; i < 4; i++){
        pid[i].change_parameters(pid_params);
        pid[i].clear_i_out();
    }
}

void EngineerChassisSKD::update_target_current() {

    if (enable){
        // FR, +vx, -vy, -w
        // FL, +vx, +vy, -w, since the motor is installed in the opposite direction
        // BL, -vx, +vy, -w, since the motor is installed in the opposite direction
        // BR, -vx, -vy, -w

        target_velocity[FR] = (+target_vx - target_vy - target_w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
        EngineerChassisIF::motors[FR].target_current = (int16_t) pid[FR].calc(EngineerChassisIF::motors[FR].actual_velocity,
                                                                               target_velocity[FR]);
//        LOG("%d",EngineerChassisIF::motors[FR].target_current);
        target_velocity[FL] = (+target_vx + target_vy - target_w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
        EngineerChassisIF::motors[FL].target_current = (int16_t) pid[FL].calc(EngineerChassisIF::motors[FL].actual_velocity,
                                                                               target_velocity[FL]);
        target_velocity[BL] = (-target_vx + target_vy - target_w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
        EngineerChassisIF::motors[BL].target_current = (int16_t) pid[BL].calc(EngineerChassisIF::motors[BL].actual_velocity,
                                                                               target_velocity[BL]);
        target_velocity[BR] = (-target_vx - target_vy - target_w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
        EngineerChassisIF::motors[BR].target_current = (int16_t) pid[BR].calc(EngineerChassisIF::motors[BR].actual_velocity,
                                                                               target_velocity[BR]);
    } else{
        EngineerChassisIF::motors[FR].target_current = EngineerChassisIF::motors[FL].target_current
                = EngineerChassisIF::motors[BL].target_current = EngineerChassisIF::motors[BR].target_current = 0;
    }
}

void EngineerChassisSKD::set_velocity(float target_vx_, float target_vy_, float target_w_) {
    target_vx = target_vx_;
    target_vy = target_vy_;
    target_w = target_w_;
}

void EngineerChassisSKD::EngineerChassisThread::main() {
    setName("EngineerChassis");
    EngineerChassisSKD::init();

    while (!shouldTerminate()){
        update_target_current();
        EngineerChassisIF::send_currents();
        sleep(TIME_MS2I(20));
    }
}
