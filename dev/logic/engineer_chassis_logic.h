//
// Created by 钱晨 on 7/29/21.
//

#ifndef META_INFANTRY_ENGINEER_CHASSIS_LOGIC_H
#define META_INFANTRY_ENGINEER_CHASSIS_LOGIC_H

#include "ahrs.h"
#include "hal.h"
#include "ch.hpp"
#include "engineer_chassis_skd.h"
#include "pid_controller.hpp"

using namespace chibios_rt;

class EngineerChassisLG {
public:

    static enum mode_t{
        FORCE_RELAX_MODE,
        NORMAL_MODE
    } mode;

    static void start(AbstractAHRS *chassis_ahrs_, const Matrix33 ahrs_angle_rotation_,
                      const Matrix33 ahrs_gyro_rotation_, PIDController::pid_params_t YAW_PID_PARAMETERS_, tprio_t LOGIC_THREAD_PRIO);

    static void set_target(float target_vx_, float target_vy_, float angle_);

    static void set_mode(mode_t mode_);

    static PIDController YAW_PID;
    static AbstractAHRS *ahrs;
    static Vector3D ahrs_angle;
    static Matrix33 ahrs_angle_rotation;
    static Matrix33 ahrs_gyro_rotation;
    static float last_yaw_angle;
    static float accumulate_yaw_angle;
    static float actual_yaw_velocity;
    static float target_vx;
    static float target_vy;
    static float target_angle;

    static class LogicThread : public BaseStaticThread<512> {
        void main() final;
    }logicThread;

};

#endif //META_INFANTRY_ENGINEER_CHASSIS_LOGIC_H
