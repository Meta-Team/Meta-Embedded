//
// Created by liuzikai on 2019-05-18.
//

#ifndef META_INFANTRY_THREAD_CHASSIS_H
#define META_INFANTRY_THREAD_CHASSIS_H

#include "ch.hpp"
#include "hal.h"

#include "remote_interpreter.h"
#include "chassis.h"

/**
 * @name ChassisThread
 * @brief thread to control chassis
 * @pre RemoteInterpreter start receive
 * @pre initialize ChassisInterface with CAN driver
 */
class ChassisThread : public chibios_rt::BaseStaticThread<1024> {

public:

    ChassisThread(Chassis::pid_params_t CHASSIS_PID_V2I_PARAMS_) : PID_V2I_PARAMS(CHASSIS_PID_V2I_PARAMS_) {};

    void enable_external_mode();

    void disable_external_mode();

    void set_external_target(float target_vx, float target_vy, float target_w);

private:

    const Chassis::pid_params_t PID_V2I_PARAMS;

    bool external_mode = false;
    float external_target_vx = 0.0f;
    float external_target_vy = 0.0f;
    float external_target_w = 0.0f;

    static constexpr unsigned int CHASSIS_THREAD_INTERVAL = 2;  // [ms]

    /**
     * Params for user
     * @note also update ONES doc
     */
    static constexpr float PC_W_VY = -500.0f;
    static constexpr float PC_S_VY = 500.0f;
    static constexpr float PC_E_VX = -500.0f;
    static constexpr float PC_Q_VX = 500.0f;
    static constexpr float PC_A_W = -100.0f;
    static constexpr float PC_D_W = 100.0f;

    static constexpr float PC_CTRL_RATIO = 0.5f;

    void main() final;
};

#endif //META_INFANTRY_THREAD_CHASSIS_H
