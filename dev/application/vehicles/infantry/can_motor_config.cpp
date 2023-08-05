//
// Created by 钱晨 on 11/14/21.
//

#include "can_motor_config.h"
//TODO Hardware sync the CAN settings on infantry 3
#if defined(INFANTRY_THREE)                                                 /** Infantry #3  (The old hardware wire connection and CANID sid, the 8/4/21 version)**/
CANMotorBase CANMotorCFG::CANMotorProfile[MOTOR_COUNT] = {
        {CANMotorBase::can_channel_2, 0x202, CANMotorBase::M3508, 3572},
        {CANMotorBase::can_channel_2, 0x201, CANMotorBase::M3508, 3572},
        {CANMotorBase::can_channel_2, 0x204, CANMotorBase::M3508, 3572},
        {CANMotorBase::can_channel_2, 0x203, CANMotorBase::M3508, 3572},

        {CANMotorBase::can_channel_2, 0x205, CANMotorBase::GM6020, 2000},//YAW
        {CANMotorBase::can_channel_1, 0x206, CANMotorBase::GM6020, 3572},//PITCH,
        // notice when GM6020's led blink once , meaning CANID = 0x204+1= 0x205, and twice 0x206, ....

        {CANMotorBase::can_channel_1, 0x207, CANMotorBase::M2006, 3572},
        {CANMotorBase::can_channel_1, 0x204, CANMotorBase::M3508_without_deceleration, 3572},
        {CANMotorBase::can_channel_1, 0x203, CANMotorBase::M3508_without_deceleration, 3572}
};

PIDController::pid_params_t CANMotorCFG::a2vParams[MOTOR_COUNT] = {
        {10, 0.0f, 0.2,  100, 500},
        {10, 0.0f, 0.2,  100, 500},
        {10, 0.0f, 0.2,  100, 500},
        {10, 0.0f, 0.2,  100, 500},
        {30.f, 0.1f, 1600.f, 000, 720},
        {30, 0.0f, 400,  000, 720},
        {20, 0.0f, 0.0,  000, 360},
        {10, 0.0f, 0.2,  100, 500},
        {10, 0.0f, 0.2,  100, 500},
};

PIDController::pid_params_t CANMotorCFG::v2iParams[MOTOR_COUNT] = {
        {26.0f,0.1f,0.02f,2000.0,6000.0},
        {26.0f,0.1f,0.02f,2000.0,6000.0},
        {26.0f,0.1f,0.02f,2000.0,6000.0},
        {26.0f,0.1f,0.02f,2000.0,6000.0},
        {10.0f,  0.15f, 0.01f, 5000.0f, 30000.0f},
        { 40.0f, 1.0f, 0.00f, 5000.0f, 30000.0f},
        {55.0f, 5.0f, 0.0f, 10000.0f, 10000.0f},
        {32.0f, 0.f, 0.02f, 5000.0f, 10000.0f},
        {32.0f, 0.f, 0.02f, 5000.0f, 10000.0f}
};

#elif defined(INFANTRY_FOUR)                                                /** Infantry #4 **/
CANMotorBase CANMotorCFG::CANMotorProfile[MOTOR_COUNT] = {
        {CANMotorBase::can_channel_2, 0x201, CANMotorBase::M3508, 3572},
        {CANMotorBase::can_channel_2, 0x202, CANMotorBase::M3508, 3572},
        {CANMotorBase::can_channel_2, 0x203, CANMotorBase::M3508, 3572},
        {CANMotorBase::can_channel_2, 0x204, CANMotorBase::M3508, 3572},
        {CANMotorBase::can_channel_2, 0x205, CANMotorBase::GM6020, 2080},//2100 increase this num let yaw turn to left
        {CANMotorBase::can_channel_1, 0x205, CANMotorBase::GM6020, 4666},
        {CANMotorBase::can_channel_1, 0x207, CANMotorBase::M2006, 3572},
        {CANMotorBase::can_channel_1, 0x204, CANMotorBase::M3508_without_deceleration, 3572},
        {CANMotorBase::can_channel_1, 0x203, CANMotorBase::M3508_without_deceleration, 3572}
};

PIDController::pid_params_t CANMotorCFG::a2vParams[MOTOR_COUNT] = {
        {10, 0, 0.2, 100, 500},
        {10, 0, 0.2, 100, 500},
        {10, 0, 0.2, 100, 500},
        {10, 0, 0.2, 100, 500},
        {22, 0, 500, 000, 550},//YAW
        {20, 0, 100, 000, 450},//pitch
        {20, 0, 0.0, 000, 360},
        {10, 0, 0.2, 100, 500},
        {10, 0, 0.2, 100, 500},
};

PIDController::pid_params_t CANMotorCFG::v2iParams[MOTOR_COUNT] = {
        {26.0f, 0.1f, 0.02f, 2000.0,   6000.0},
        {26.0f, 0.1f, 0.02f, 2000.0,   6000.0},
        {26.0f, 0.1f, 0.02f, 2000.0,   6000.0},
        {26.0f, 0.1f, 0.02f, 2000.0,   6000.0},
        {15.0f, 0,    5.0f,  5000.0f,  30000.0f},//YAW
        {40.0f, 0, 5.0f, 5000.0f,  30000.0f},
        {55.0f, 5.0f, 0.0f,  10000.0f, 10000.0f},
        {32.0f, 0.f,  0.02f, 5000.0f,  10000.0f},
        {32.0f, 0.f,  0.02f, 5000.0f,  10000.0f}
};

#endif


bool CANMotorCFG::enable_a2v[MOTOR_COUNT]{
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        false
};

bool CANMotorCFG::enable_v2i[MOTOR_COUNT]{
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        false
};