//
// Created by 钱晨 on 10/29/21.
//

#include "Inspector.h"
#include "led.h"
void Inspector::startup_check_motor() {
    time_msecs_t t = SYSTIME;
    while (WITHIN_RECENT_TIME(t, 20)) {
        for (auto & i : CANMotorInterface::motor_feedback) {
            if (not WITHIN_RECENT_TIME(i.last_update_time, 5)) {
                // No feedback in last 5 ms (normal 1 ms)
                LOG_ERR("Startup - Motor offline.");
                t = SYSTIME;  // reset the counter
            }
        }
    }
}

void Inspector::startup_check_VCP() {
    time_msecs_t t = SYSTIME;
    while (WITHIN_RECENT_TIME(t, 20)) {
        if(not WITHIN_RECENT_TIME(VirtualCOMPort::last_update_time, 10)) {
            LOG_ERR("Startup - Virtual COM port not connected");
            t = SYSTIME;
        }
    }
}

