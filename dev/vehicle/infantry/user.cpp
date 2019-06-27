//
// Created by liuzikai on 2019-06-25.
//

#include "user.h"

User::UserThread User::userThread;

void User::start(tprio_t prio) {
    userThread.start(prio);
}

void User::UserThread::main() {
    setName("User");
    while(!shouldTerminate()) {

        /// Chassis
        if (!Inspector::remote_failure() && !Inspector::chassis_failure()) {

            if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) {

                Chassis::calc(0,
                              -Remote::rc.ch3 * COMMON_VY,
                              Remote::rc.ch2 * COMMON_W);
                ChassisLG::set_action(ChassisLG::FOLLOW_MODE);


            } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_DOWN) {

                Chassis::calc(-Remote::rc.ch2 * COMMON_VX,
                              -Remote::rc.ch3 * COMMON_VY,
                              Remote::rc.ch0 * COMMON_W);

            } else if (Remote::rc.s1 == Remote::S_DOWN) { // PC control mode

                // Determine target velocities

                float target_vx, target_vy, target_w;

                /** NOTICE: here the minus sign make it incoherent with the initial definition of chassis coordinate */

                if (Remote::key.w) target_vy = -COMMON_VY;
                else if (Remote::key.s) target_vy = COMMON_VY;
                else target_vy = 0;

                if (Remote::key.q) target_vx = -COMMON_VX;
                else if (Remote::key.e) target_vx = COMMON_VX;
                else target_vx = 0;

                if (Remote::key.a) target_w = -COMMON_W;
                else if (Remote::key.d) target_w = COMMON_W;
                else target_w = 0;

                if (Remote::key.ctrl) {
                    target_vx *= PC_CTRL_RATIO;
                    target_vy *= PC_CTRL_RATIO;
                    target_w *= PC_CTRL_RATIO;
                }

                Chassis::calc(target_vx, target_vy, target_w);

            } else {

                for (int i = 0; i < Chassis::MOTOR_COUNT; i++) {
                    Chassis::target_current[i] = 0;
                }

            }

        } else {  // StateHandler::remoteDisconnected() || StateHandler::gimbalSeriousErrorOccured()

            for (int i = 0; i < Chassis::MOTOR_COUNT; i++) {
                Chassis::target_current[i] = 0;
            }

        }
        Chassis::send_chassis_currents();

        sleep(TIME_MS2I(chassis_thread_interval));

        /// Final
        sleep(TIME_MS2I(USER_THREAD_INTERVAL));
    }
}