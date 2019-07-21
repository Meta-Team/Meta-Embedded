//
// Created by Kerui Zhu on 7/9/2019.
//

#include "engineer_chassis_skd.h"

EngineerChassisSKD::EngineerChassisThread EngineerChassisSKD::engineerChassisThread;
float EngineerChassisSKD::target_velocity[ENGINEER_CHASSIS_MOTOR_COUNT];
bool EngineerChassisSKD::enable;
bool EngineerChassisSKD::time_control;
time_msecs_t EngineerChassisSKD::test_end_time;
float EngineerChassisSKD::target_vx;
float EngineerChassisSKD::target_vy;
float EngineerChassisSKD::target_w;
PIDController EngineerChassisSKD::pid[ENGINEER_CHASSIS_MOTOR_COUNT];

void EngineerChassisSKD::init(){
    lock();
    time_control = false;
    test_end_time = 0;
    change_pid_params(CHASSIS_PID_V2I_PARAMS);
}

void EngineerChassisSKD::lock() {
    if (enable) {
        enable = false;
        target_vx = target_vy = target_w = 0;
    }
}

void EngineerChassisSKD::unlock() {
    if (!enable) {
        enable = true;
        set_velocity(0,0,0);
        for (PIDController pidController : pid) pidController.clear_i_out();
    }
}

void EngineerChassisSKD::set_test_end_time(time_msecs_t run_time) {
    time_control = true;
    test_end_time = TIME_I2MS(chVTGetSystemTime()) + run_time;
}

void EngineerChassisSKD::change_pid_params(PIDControllerBase::pid_params_t pid_params) {
    for (int i = 0; i < 4; i++){
        pid[i].change_parameters(pid_params);
        pid[i].clear_i_out();
    }
}

void EngineerChassisSKD::print_pid() {
    PIDControllerBase::pid_params_t to_print = pid[0].get_parameters();
    LOG("%f %f %f %f %f", to_print.kp, to_print.ki, to_print.kd, to_print.i_limit, to_print.out_limit);
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

void EngineerChassisSKD::pivot_turn(engr_motor_id_t id, float w) {
    double point_x;
    double point_y;

    if (id == FR) {
        point_x = 0.5 * CHASSIS_WHEEL_TREAD;
        point_y = 0.5 * CHASSIS_WHEEL_BASE;
    } else if (id == FL) {
        point_x = -0.5 * CHASSIS_WHEEL_TREAD;
        point_y = 0.5 * CHASSIS_WHEEL_BASE;
    } else if (id == BL) {
        point_x = -0.5 * CHASSIS_WHEEL_TREAD;
        point_y = -0.5 * CHASSIS_WHEEL_BASE;
    } else {
        point_x = 0.5 * CHASSIS_WHEEL_TREAD;
        point_y = -0.5 * CHASSIS_WHEEL_BASE;
    }

    float vx = point_y * target_w;
    float vy = -point_x * target_w;
    set_velocity(vx, vy, w);
}

void EngineerChassisSKD::EngineerChassisThread::main() {
    setName("EngineerChassis");
    EngineerChassisSKD::init();

    while (!shouldTerminate()){

        // if under time_control mode, mainly designed for matlab params adjusting
        if (time_control) {
            if (SYSTIME >= test_end_time) {
                target_vx = target_vy = target_w = 0.0f;
                Shell::printf("!ce" SHELL_NEWLINE_STR);
            }
        }

        update_target_current();
        EngineerChassisIF::send_currents();
        sleep(TIME_MS2I(2));
    }
}
