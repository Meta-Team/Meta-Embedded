//
// Created by admin on 2018/12/8.
//

#ifndef META_INFANTRY_GIMBAL_PROCESS_H
#define META_INFANTRY_GIMBAL_PROCESS_H

#include <stdio.h>
#include <stdint.h>
#include "ch.hpp"
#include "hal.h"

class gimbal_process {
public:
    uint8_t data8[8];
    typedef struct {
        uint16_t actual_angle_orig;
        int16_t actual_angle_base_round; // The number of round(s) that motor has rotated related to original position
        int16_t actual_angle;
        int16_t delta_angle;
        int16_t target_angle;
        int16_t target_current;
    } gimbal_motor;

    gimbal_motor motor[2];

    uint16_t gimbal_fi_orig[2];

    void process_gimbal_feedback(CANRxFrame *rxmsg);
};

#endif //META_INFANTRY_GIMBAL_PROCESS_H
