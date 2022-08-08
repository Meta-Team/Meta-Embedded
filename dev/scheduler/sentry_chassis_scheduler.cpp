//
// Created by zhukerui on 2019/4/29.
// Modified by jintengjun on 2019/6/11
// Modified by Laixinyi on 2019/7/1
// Modified by Chen Qian and Feiyang Wu on JUL,AUG 2022.
//

#include "sentry_chassis_scheduler.h"

SChassisSKD::mode_t SChassisSKD::mode = FORCED_RELAX_MODE;
SChassisSKD::install_direction_t SChassisSKD::motor_install_[2];

float SChassisSKD::target_position = 0;  // [cm], bigger position will drive the chassis towards right
float SChassisSKD::target_velocity = 0;  // [cm/s], calculated target velocity, middle value

SChassisSKD::SKDThread SChassisSKD::skdThread;


void SChassisSKD::start(SChassisSKD::install_direction_t left_motor_install,
                        SChassisSKD::install_direction_t right_motor_install, tprio_t thread_prio) {

    motor_install_[SChassisSKD::MOTOR_LEFT] = left_motor_install;
    motor_install_[SChassisSKD::MOTOR_RIGHT] = right_motor_install;

    skdThread.start(thread_prio);
}

void SChassisSKD::reset_origin() {
    CANMotorIF::motor_feedback[CANMotorCFG::MOTOR_LEFT].reset_accumulate_angle();
    CANMotorIF::motor_feedback[CANMotorCFG::MOTOR_RIGHT].reset_accumulate_angle();
    target_position = 0;
}

void SChassisSKD::set_mode(SChassisSKD::mode_t mode_) {
    mode = mode_;
    switch (mode) {
        case FORCED_RELAX_MODE:
            CANMotorCFG::enable_a2v[CANMotorCFG::MOTOR_LEFT] =
            CANMotorCFG::enable_a2v[CANMotorCFG::MOTOR_RIGHT] = false;
            break;
        case VELOCITY_MODE:
            CANMotorCFG::enable_a2v[CANMotorCFG::MOTOR_LEFT] =
            CANMotorCFG::enable_a2v[CANMotorCFG::MOTOR_RIGHT] = false;
            CANMotorCFG::enable_v2i[CANMotorCFG::MOTOR_LEFT] =
            CANMotorCFG::enable_v2i[CANMotorCFG::MOTOR_RIGHT] = true;
            break;
        case POSITION_MODE:
            CANMotorCFG::enable_a2v[CANMotorCFG::MOTOR_LEFT] =
            CANMotorCFG::enable_a2v[CANMotorCFG::MOTOR_RIGHT] =
            CANMotorCFG::enable_v2i[CANMotorCFG::MOTOR_LEFT] =
            CANMotorCFG::enable_v2i[CANMotorCFG::MOTOR_RIGHT] = true;
            break;
    }
}

void SChassisSKD::set_destination(float dist) {
    target_position = dist;
}

void SChassisSKD::set_velocity(float velocity) {
    target_velocity = velocity;
}

float SChassisSKD::present_position() {
    return CANMotorIF::motor_feedback[CANMotorCFG::MOTOR_LEFT].accumulate_angle()
           * (float) motor_install_[0]
           * schassis_wheel_curriculum;
}

float SChassisSKD::present_velocity() {
    return CANMotorIF::motor_feedback[CANMotorCFG::MOTOR_LEFT].actual_velocity
           * (float) motor_install_[0]
           * schassis_wheel_curriculum;
}

void SChassisSKD::SKDThread::main() {
    setName("SChassis_SKD");
    while (!shouldTerminate()) {
        if (mode == POSITION_MODE) {

            chSysLock();
            // TODO: Using optical sensor to get more accurate movement data. Register the data to the
            //       CAN motor controller by calling CANMotorController::register_custom_feedback().
            CANMotorController::set_target_angle(CANMotorCFG::MOTOR_LEFT,
                                                 target_position / schassis_wheel_curriculum
                                                 * (float)motor_install_[SChassisSKD::MOTOR_LEFT]);
            CANMotorController::set_target_angle(CANMotorCFG::MOTOR_RIGHT,
                                                 target_position / schassis_wheel_curriculum
                                                 * (float)motor_install_[SChassisSKD::MOTOR_RIGHT]);
            chSysUnlock();

        } else if (mode == VELOCITY_MODE) {

            chSysLock();
            CANMotorController::set_target_vel(CANMotorCFG::MOTOR_LEFT,
                                                 target_velocity / schassis_wheel_curriculum
                                                 * (float)motor_install_[SChassisSKD::MOTOR_LEFT]);
            CANMotorController::set_target_vel(CANMotorCFG::MOTOR_RIGHT,
                                               target_velocity / schassis_wheel_curriculum
                                                 * (float)motor_install_[SChassisSKD::MOTOR_RIGHT]);
            chSysUnlock();

        } else if (mode == FORCED_RELAX_MODE) {

            if(CANMotorCFG::enable_v2i[CANMotorCFG::MOTOR_LEFT]) {

                // Prevent a2v pid controller overwriting velocity target.
                for (int i = CANMotorCFG::MOTOR_LEFT; i < CANMotorCFG::MOTOR_RIGHT + 1; i++) {
                    CANMotorCFG::enable_a2v[i] = false;
                }

                // Set velocity to 0
                while (!ABS_IN_RANGE(CANMotorIF::motor_feedback[CANMotorCFG::MOTOR_LEFT].actual_velocity,  5.0) &&
                       !ABS_IN_RANGE(CANMotorIF::motor_feedback[CANMotorCFG::MOTOR_RIGHT].actual_velocity, 5.0)) {
                    for (int i = 0; i < CANMotorCFG::MOTOR_COUNT; i++) {
                        CANMotorController::set_target_vel((CANMotorCFG::motor_id_t) i, 0.0f);
                    }
                    sleep(TIME_MS2I(50));
                }
            }

            // Prevent v2i pid controller overwriting torque current target.
            for (int i = CANMotorCFG::MOTOR_LEFT; i < CANMotorCFG::MOTOR_RIGHT + 1; i++) {
                CANMotorCFG::enable_v2i[i] = false;
            }

            // Set target current to 0.
            for (int i = CANMotorCFG::MOTOR_LEFT; i < CANMotorCFG::MOTOR_RIGHT + 1; i++) {
                CANMotorController::set_target_current((CANMotorCFG::motor_id_t)i, 0);
            }
        }

        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));
    }
}