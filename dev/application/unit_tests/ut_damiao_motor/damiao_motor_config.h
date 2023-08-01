//
// Created by Wu Feiyang on 7/13/23.
//

#ifndef META_DAMIAO_MOTOR_CFG_H
#define META_DAMIAO_MOTOR_CFG_H

/**
 * Damiao 4310 motor.
 */
#include "hal.h"
#define can_channel_1 &CAND1
#define can_channel_2 &CAND2

typedef enum motor_mode{
    MIT_MODE,
    POS_VEL_MODE,
    VEL_MODE,
}motor_mode_t;

class DamiaoMotorBase{
public:
    CANDriver* can_driver;
    int        masterID;
    int        slaveID;
    float      mitKp;
    float      mitKd;
    float      V_max;   // maximum rotation speed. Unit is Rad/s.
    float      P_max;   // maximum Position. Unit is Rad.
    float      T_max;   // maximum Torque. Unit is N*m.
    float      initial_encoder_angle;
    motor_mode_t mode;
    float      kp_min;
    float      kp_max;
    float      kd_min;
    float      kd_max;
};

class DamiaoMotorCFG{
public:
    enum MotorName{
        YAW,
        PITCH,
        MOTOR_COUNT,
    }motor_usage_t;
    /***
     * @brief Motor Configuration
     * The unit of all velocity is Rad/s, unit of P_MAX must be exactly identical to the value with that of the Damiao Offical tool,
     * whose unit is by defualt Rad. All unit of torque is N * m. The unit of @param initial_encoder_angle is degree.
     */
    static constexpr DamiaoMotorBase motorCfg[MOTOR_COUNT] = {
            {can_channel_1,0x00,0x01,1.0,0.3,30,3.141953,10.0,
             55.26,POS_VEL_MODE,0.0,500.0,0.0,5.0},
            {can_channel_2,0x00,0x01,0.0,0.0,30,3.141593,10.0,
             0.0,VEL_MODE,0.0,500.0,0.0,5.0}
    };
};

#endif //META_DAMIAO_MOTOR_CFG_H
