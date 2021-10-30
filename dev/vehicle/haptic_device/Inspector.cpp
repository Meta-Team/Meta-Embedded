//
// Created by 钱晨 on 10/29/21.
//

#include "Inspector.h"
#include "led.h"
void Inspector::startup_check_motor() {
    time_msecs_t t = SYSTIME;
    while (WITHIN_RECENT_TIME(t, 20)) {
        for (auto & i : can_motor_interface::motor_feedback) {
            if (not WITHIN_RECENT_TIME(i.last_update_time, 5)) {
                // No feedback in last 5 ms (normal 1 ms)
                LOG_ERR("Startup - Motor offline.");
                t = SYSTIME;  // reset the counter
            }
        }
    }
}
