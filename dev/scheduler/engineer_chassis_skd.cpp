//
// Created by Kerui Zhu on 7/9/2019.
//

#include "engineer_chassis_skd.h"
#include "math.h"

float EngineerChassisSKD::target_velocity[MOTOR_COUNT];
bool EngineerChassisSKD::enabled;

float EngineerChassisSKD::target_vx;
float EngineerChassisSKD::target_vy;
float EngineerChassisSKD::target_w;

float EngineerChassisSKD::wheel_base_ = 0;
float EngineerChassisSKD::wheel_tread_ = 0;
float EngineerChassisSKD::wheel_circumference_ = 0;
float EngineerChassisSKD::w_to_v_ratio_ = 0.0f;
float EngineerChassisSKD::v_to_wheel_angular_velocity_ = 0.0f;

PIDController EngineerChassisSKD::pid[MOTOR_COUNT];

EngineerChassisSKD::SKDThread EngineerChassisSKD::skdThread;


void EngineerChassisSKD::start(float wheel_base, float wheel_tread, float wheel_circumference, tprio_t thread_prio) {

    wheel_base_ = wheel_base;
    wheel_tread_ = wheel_tread;
    wheel_circumference_ = wheel_circumference;

    w_to_v_ratio_ = (wheel_base + wheel_tread) / 2.0f / 180.0f * M_PI;
    v_to_wheel_angular_velocity_ = (360.0f / wheel_circumference);

    skdThread.start(thread_prio);
}

void EngineerChassisSKD::enable(bool enable_) {
    enabled = enable_;
}

bool EngineerChassisSKD::is_locked() {
    return (!enabled);
}

void EngineerChassisSKD::load_pid_params(PIDControllerBase::pid_params_t v2i_pid_params) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        ///pid was declared as: PIDController EngineerChassisSKD::pid[MOTOR_COUNT];
        pid[i].change_parameters(v2i_pid_params);
        pid[i].clear_i_out();
    }
}

void EngineerChassisSKD::set_velocity(float target_vx_, float target_vy_, float target_w_) {
    ///modify the static variables of Class EngineerChassisSKD according to function input
    target_vx = target_vx_;
    target_vy = target_vy_;
    target_w = target_w_;
}

void EngineerChassisSKD::pivot_turn(float x, float y, float w) {
    set_velocity(w * M_PI / 180.0f * y, - w * M_PI / 180.0f * x, w);
}

void EngineerChassisSKD::SKDThread::main() {
    setName("EChasssisSKD");

    while (!shouldTerminate()) {

        if (enabled) {
            ///Calculate the target velocity concretely for each motor according to the target velocity of the whole car
            ///Specify the target current of each motor by v2i PID: pid[i].change_parameters(v2i_pid_params);
            // FR, +vx, -vy, -w
            // FL, +vx, +vy, -w, since the motor is installed in the opposite direction
            // BL, -vx, +vy, -w, since the motor is installed in the opposite direction
            // BR, -vx, -vy, -w

            target_velocity[FR] = (+target_vx - target_vy - target_w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
            ChassisIF::target_current[FR] = (int16_t) pid[FR].calc(ChassisIF::feedback[FR].actual_velocity,
                                                                   target_velocity[FR]);
            target_velocity[FL] = (+target_vx + target_vy - target_w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
            ChassisIF::target_current[FL] = (int16_t) pid[FL].calc(ChassisIF::feedback[FL].actual_velocity,
                                                                   target_velocity[FL]);
            target_velocity[BL] = (-target_vx + target_vy - target_w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
            ChassisIF::target_current[BL] = (int16_t) pid[BL].calc(ChassisIF::feedback[BL].actual_velocity,
                                                                   target_velocity[BL]);
            target_velocity[BR] = (-target_vx - target_vy - target_w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
            ChassisIF::target_current[BR] = (int16_t) pid[BR].calc(ChassisIF::feedback[BR].actual_velocity,
                                                                   target_velocity[BR]);

        } else {

            for (size_t i = 0; i < MOTOR_COUNT; i++) {
                ChassisIF::target_current[i] = 0;
            }

        }

        ChassisIF::enable_chassis_current_clip();
        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));
    }
}
