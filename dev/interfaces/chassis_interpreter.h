//
// Created by 钱晨 on 2019-01-10.

#ifndef INSOULED_CHIBIOS_CHASSIS_FEEDBACK_H
#define INSOULED_CHIBIOS_CHASSIS_FEEDBACK_H

#include "ch.hpp"
#include "hal.h"
#include "can_interface.h"


class chassis_interpreter
{
    public:
    typedef struct
    {
        uint16_t actual_angle;
        int16_t actual_rpm;
        int16_t actual_current;
        uint8_t actual_temperature;
        float actual_angular_velocity;
    } motor_info;
    typedef struct
    {
        float vx;
        float vy;
        float w;
        motor_info motor[4];
    } chassis_info;
    chassis_info chassis_inf;
    void process_chassis_feedback(CANRxFrame *rxmsg); // Get the frame from the CAN. Will calculate the vx, vy, w in latter version.
    private:


};
#endif //INSOULED_CHIBIOS_CHASIS_FEEDBACK_H