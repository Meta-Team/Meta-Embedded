//
// Created by 钱晨 on 7/29/21.
//

#include "engineer_chassis_logic.h"

float EngineerChassisLG::target_vy = 0.0f;
float EngineerChassisLG::target_angle = 0.0f;
float EngineerChassisLG::target_vx = 0.0f;
AbstractAHRS *EngineerChassisLG::ahrs;
EngineerChassisLG::LogicThread EngineerChassisLG::logicThread;
Matrix33 EngineerChassisLG::ahrs_angle_rotation;
Matrix33 EngineerChassisLG::ahrs_gyro_rotation;
PIDController EngineerChassisLG::YAW_PID;
float EngineerChassisLG::last_yaw_angle;
float EngineerChassisLG::accumulate_yaw_angle;
float EngineerChassisLG::actual_yaw_velocity;
EngineerChassisLG::mode_t EngineerChassisLG::mode;


void EngineerChassisLG::start(AbstractAHRS *chassis_ahrs_, const Matrix33 ahrs_angle_rotation_,
                              const Matrix33 ahrs_gyro_rotation_, PIDController::pid_params_t YAW_PID_PARAMETERS_, tprio_t LOGIC_THREAD_PRIO){
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ahrs_angle_rotation[i][j] = ahrs_angle_rotation_[i][j];
            ahrs_gyro_rotation[i][j] = ahrs_gyro_rotation_[i][j];
        }
    }
    ahrs = chassis_ahrs_;
    YAW_PID.change_parameters(YAW_PID_PARAMETERS_);
    logicThread.start(LOGIC_THREAD_PRIO);
}

void EngineerChassisLG::set_target(float target_vx_, float target_vy_, float angle_) {
    target_vx = target_vx_;
    target_vy = target_vy_;
    target_angle = angle_;
}

void EngineerChassisLG::set_mode(mode_t mode_) {
    mode = mode_;
}

void EngineerChassisLG::LogicThread::main() {
    // Fetch data
    Vector3D ahrs_angle = ahrs->get_angle() * ahrs_angle_rotation;
    Vector3D ahrs_gyro = ahrs->get_gyro() * ahrs_gyro_rotation;

    // TODO: document calculations here
    float angle[2] = {ahrs_angle.x, ahrs_angle.y};
    float velocity[2] = {
            ahrs_gyro.x * cosf(ahrs_angle.y / 180.0f * M_PI) + ahrs_gyro.z * sinf(ahrs_angle.y / 180.0f * M_PI),
            ahrs_gyro.y};


    float yaw_angle_movement = angle[0] - last_yaw_angle;
    last_yaw_angle = angle[0];

    if (yaw_angle_movement < -200) yaw_angle_movement += 360;
    if (yaw_angle_movement > 200) yaw_angle_movement -= 360;

    // Use increment to calculate accumulated angles
    accumulate_yaw_angle += yaw_angle_movement;
    actual_yaw_velocity = velocity[0];

    if(mode == FORCE_RELAX_MODE) {
        EngineerChassisSKD::enable(false);
    } else if(mode == NORMAL_MODE){
        EngineerChassisSKD::enable(true);
        YAW_PID.calc(actual_yaw_velocity, target_angle);
        EngineerChassisSKD::set_velocity(target_vx, target_vy, YAW_PID.calc(actual_yaw_velocity, target_angle));
    }
    sleep(TIME_MS2I(5));
}