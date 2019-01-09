//
// Created by liuzikai on 2019-01-02.
//

#include "pid_controller.h"

float PIDController::calc(float now, float target) {

    error[1] = error[0];
    error[0] = target - now;

    p_out = kp * error[0];
    i_out += ki * error[0];
    d_out = kd * (error[0] - error[1]);

    if (i_out > i_limit) i_out = i_limit;
    if (i_out < -i_limit) i_out = -i_limit;

    out = p_out + i_out + d_out;
    if (out > out_limit) out = out_limit;
    if (out < -out_limit) out = -out_limit;

    return out;
}