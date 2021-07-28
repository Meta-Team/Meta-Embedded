//
// Created by liuzikai on 7/29/21.
//

#include "pa_user_gimbal.h"

PAUserGimbal::UserThread PAUserGimbal::user_thread;

static constexpr float MAX_VELOCITY[3] = {30, 10, 10};
static constexpr float MAX_ANGLE[3] = {90, 30, 0};
static constexpr float MIN_ANGLE[3] = {-90, -10, -30};

void PAUserGimbal::start(tprio_t prio) {
    user_thread.start(prio);
}

void PAUserGimbal::UserThread::main() {
    setName("PAUserGimbal");
    while (!shouldTerminate()) {

        if (Remote::rc.s1 == Remote::S_UP) {
            // Safety mode
            GimbalSKD::set_mode(GimbalSKD::FORCED_RELAX_MODE);
            for (bool &e : GimbalSKD::motor_enable) e = false;
        } else {
            GimbalSKD::set_mode(GimbalSKD::ENABLED_MODE);

            int id = (int) Remote::rc.s2 - 1;
            for (int i = 0; i < 3; i++) GimbalSKD::motor_enable[i] = (id == i);

            if (Remote::rc.s1 == Remote::S_MIDDLE) {
                // Adjust V2I
                GimbalSKD::a2v_pid_enabled = false;
                GimbalSKD::target_velocity[id] = Remote::rc.ch1 * MAX_VELOCITY[id];
            } else {
                // Adjust A2V
                GimbalSKD::a2v_pid_enabled = true;
                GimbalSKD::target_angle[id] = Remote::rc.ch1 * (Remote::rc.ch1 >= 0 ? MAX_ANGLE[id] : -MIN_ANGLE[id]);
            }
        }

        sleep(TIME_MS2I(10));
    }
}
