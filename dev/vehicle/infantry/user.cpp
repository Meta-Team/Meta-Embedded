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
    while (!shouldTerminate()) {

        /// Chassis
        if (!Inspector::remote_failure() && !Inspector::chassis_failure()) {

            if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_UP) {

                // Remote - Chassis Move + Chassis Follow
                ChassisLG::set_action(ChassisLG::FOLLOW_MODE);
                ChassisLG::set_target(Remote::rc.ch2 * CHASSIS_COMMON_VX,  // Both use right as positive direction
                                      Remote::rc.ch3 * CHASSIS_COMMON_VX   // Both use up    as positive direction
                );

            } else if (Remote::rc.s1 == Remote::S_MIDDLE && Remote::rc.s2 == Remote::S_MIDDLE) {

                // Remote - Chassis Move + Chassis Dodge
                ChassisLG::set_action(ChassisLG::DODGE_MODE);
                ChassisLG::set_target(Remote::rc.ch2 * CHASSIS_COMMON_VX,  // Both use right as positive direction
                                      Remote::rc.ch3 * CHASSIS_COMMON_VX   // Both use up    as positive direction
                );
            } else if (Remote::rc.s1 == Remote::S_DOWN) {

                // PC control mode

                if (ChassisLG::get_action() == ChassisLG::STOP_MODE) {
                    // Enter PC Mode from other mode, re-enable ChassisLG
                    ChassisLG::set_action(ChassisLG::FOLLOW_MODE);
                }

                if (chassis_x_pressed != Remote::key.x) {  // key x is pressed or released
                    if (Remote::key.x) {  // enter dodge mode
                        ChassisLG::set_action(ChassisLG::DODGE_MODE);
                    } else {  // exit dodge mode
                        ChassisLG::set_action(ChassisLG::FOLLOW_MODE);
                    }
                    chassis_x_pressed = Remote::key.x;
                }

                float target_vx, target_vy;

                if (Remote::key.w) target_vy = CHASSIS_COMMON_VY;
                else if (Remote::key.s) target_vy = -CHASSIS_COMMON_VY;
                else target_vy = 0;

                if (Remote::key.d) target_vx = CHASSIS_COMMON_VY;
                else if (Remote::key.a) target_vx = -CHASSIS_COMMON_VY;
                else target_vx = 0;

                if (Remote::key.ctrl) {
                    target_vx *= CHASSIS_PC_CTRL_RATIO;
                    target_vy *= CHASSIS_PC_CTRL_RATIO;
                }

                ChassisLG::set_target(target_vx, target_vy);

            } else {
                // Safe Mode
                ChassisLG::set_action(ChassisLG::STOP_MODE);
            }

        } else {  // Inspector::remote_failure() || Inspector::chassis_failure()
            // Safe Mode
            ChassisLG::set_action(ChassisLG::STOP_MODE);
        }


        /// Final
        sleep(TIME_MS2I(USER_THREAD_INTERVAL));
    }
}