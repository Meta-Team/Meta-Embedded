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
        int16_t actual_angle_base_round; // The number of round(s) that motor has rotated related to original position
        float actual_angle;
    } gimbal_motor;

    gimbal_motor motor[2];

    uint16_t gimbal_fi_orig[2];

    void process_gimbal_feedback(CANRxFrame *rxmsg);
};

#endif //META_INFANTRY_GIMBAL_PROCESS_H
