//
// Created by liuzikai on 2019-06-25.
//

#include "user_engineer.h"

/// Chassis Config
float UserE::chassis_v_left_right = 800.0f;  // [mm/s]
float UserE::chassis_v_forward = 1000.0f;     // [mm/s]
float UserE::chassis_v_backward = 1000.0f;    // [mm/s]
float UserE::chassis_w = 90.0f;    // [degree/s]

float UserE::chassis_pc_shift_ratio = 1.5f;  // 150% when Shift is pressed
float UserE::chassis_pc_ctrl_ratio = 0.2;    // 20% when Ctrl is pressed

/// Variables
float UserE::gimbal_pc_yaw_target_angle_ = 0;
float UserE::gimbal_pc_pitch_target_angle_ = 0;
UserE::UserThread UserE::userThread;
UserE::UserActionThread UserE::userActionThread;
UserE::ClientDataSendingThread UserE::clientDataSendingThread;

float grab_target_angle = 0.0f;
bool grabber_holded = false;

void UserE::start(tprio_t user_thd_prio, tprio_t user_action_thd_prio, tprio_t client_data_sending_thd_prio) {
    userThread.start(user_thd_prio);
    userActionThread.start(user_action_thd_prio);
    clientDataSendingThread.start(client_data_sending_thd_prio);

    // Normal speed, 2 lights from right
}


/**
 * Mode Table:
 * ------------------------------------------------------------
 * Left  Right   Mode
 * ------------------------------------------------------------
 *  UP    UP/MID Safe
 *  UP    DOWN   Sensors & Swithces Test Mode
 *  MID   UP     Remote - Chassis remote controlling
 *  MID   MID    Remote - Elevator remote controlling
 *  MID   DOWN   Remote - Gimbal remote controlling
 *  DOWN  *      PC     - AUTO_ELEVATING
 *  -Others-     Safe
 * ------------------------------------------------------------
 */

void UserE::UserThread::main() {
    setName("UserE");
    float gimbal_yaw_target_angle_ = 0.0f;
    float gimbal_rc_yaw_max_speed = 180.0f;
    while (!shouldTerminate()) {

        /// Sensors and Switches Test
        if (Remote::rc.s1 == Remote::S_MIDDLE) {
            if(Remote::rc.s2 == Remote::S_UP) {
                EngineerChassisLG::set_mode(EngineerChassisLG::NORMAL_MODE);
                EngineerChassisSKD::set_velocity(Remote::rc.ch2 * chassis_v_left_right,  // Both use right as positive direction
                                                 (Remote::rc.ch3 > 0 ?
                                                  Remote::rc.ch3 * 3000 :
                                                  Remote::rc.ch3 * 1600), -Remote::rc.ch0 * gimbal_rc_yaw_max_speed * USER_THREAD_INTERVAL / 12);
                if(Remote::rc.ch1 >= 0.5) {
                    //EngineerElevatorIF::set_elevator(EngineerElevatorIF::UP, 1000);
                    EngineerElevatorIF::set_elevator(EngineerElevatorIF::UP, 600);
                } else if (Remote::rc.ch1 <= (-0.5)) {
                    EngineerElevatorIF::set_elevator(EngineerElevatorIF::DOWN, 600);
                    //EngineerElevatorSKD::set_target_height(000.0f);
                } else {
                    EngineerElevatorIF::set_elevator(EngineerElevatorIF::STOP, 0);
                }
            } else if(Remote::rc.s2 == Remote::S_MIDDLE) {
                EngineerChassisLG::set_mode(EngineerChassisLG::NORMAL_MODE);
                EngineerChassisSKD::set_velocity(Remote::rc.ch2 * chassis_v_left_right,  // Both use right as positive direction
                                                 (Remote::rc.ch3 > 0 ?
                                                  Remote::rc.ch3 * 3000 :
                                                  Remote::rc.ch3 * 1600), -Remote::rc.ch0 * gimbal_rc_yaw_max_speed * USER_THREAD_INTERVAL / 12);
                grab_target_angle += Remote::rc.ch1*1.0f;
                VAL_CROP(grab_target_angle, 90.0f, -90.0f);
                EngineerGrabSKD::set_angle(grab_target_angle);
            } else if(Remote::rc.s2 == Remote::S_DOWN) {
                if(Remote::rc.ch0 > 0.5) {
                    palWritePad(GPIOH, GPIOH_POWER1_CTRL, PAL_HIGH);
                } else {
                    palWritePad(GPIOH, GPIOH_POWER1_CTRL, PAL_LOW);
                }
                if(Remote::rc.ch1 > 0.5) {
                    palWritePad(GPIOH, GPIOH_POWER2_CTRL, PAL_HIGH);
                } else {
                    palWritePad(GPIOH, GPIOH_POWER2_CTRL, PAL_LOW);
                }
                if(Remote::rc.ch2 > 0.5) {
                    palWritePad(GPIOH, GPIOH_POWER3_CTRL, PAL_HIGH);
                } else {
                    palWritePad(GPIOH, GPIOH_POWER3_CTRL, PAL_LOW);
                }
                if(Remote::rc.ch3 > 0.5) {
                    palWritePad(GPIOH, GPIOH_POWER4_CTRL, PAL_HIGH);
                } else {
                    palWritePad(GPIOH, GPIOH_POWER4_CTRL, PAL_LOW);
                }
            }

            // ch0 use right as positive direction, while GimbalLG use CCW (left) as positive direction
        } else if (Remote::rc.s1 == Remote::S_DOWN) {
            EngineerChassisLG::set_mode(EngineerChassisLG::NORMAL_MODE);
                /**
                 * PC Key Table:
                 * ------------------------------------------------------------
                 * Key      Function
                 * ------------------------------------------------------------
                 * QWES     LEFT, UP, RIGHT, DOWN
                 * AD       CCW, CW
                 * RF       PAUSE + UP-STAIR, DOWN-STAIR
                 * CTRL     SLOW MOVEMENT
                 * SHIFT    FAST MOVEMENT
                 * V        OPEN/CLOSE DOOR
                 * C        TURN AROUND
                 * MOUSE_L  ROBOTIC_ARM NEXT STEP
                 * MOUSE_R  ROBOTIC_ARM PREV STEP
                 * ------------------------------------------------------------
                 */

                /// Chassis WSQE, AD, ctrl, shift
                float target_vx, target_vy;

                if (Remote::key.w) {
                    target_vy = chassis_v_forward;
                } else if (Remote::key.s) target_vy = -chassis_v_backward;
                else target_vy = 0;

                if (Remote::key.a) target_vx = -chassis_v_left_right;
                else if (Remote::key.d) target_vx = chassis_v_left_right;
                else target_vx = 0;

            float yaw_sensitivity, pitch_sensitivity;
            yaw_sensitivity = 5000000;
            // Referee client data will be sent by ClientDataSendingThread

            float yaw_delta = -Remote::mouse.x * (yaw_sensitivity * USER_THREAD_INTERVAL / 1000.0f);

            VAL_CROP(yaw_delta, 50, -50);

//            gimbal_pc_yaw_target_angle_ += yaw_delta;

            if (Remote::key.ctrl) {
                target_vx *= chassis_pc_ctrl_ratio;
                target_vy *= chassis_pc_ctrl_ratio;
            } else if (Remote::key.shift) {
                target_vx *= chassis_pc_shift_ratio;
                target_vy *= chassis_pc_shift_ratio;
            } else {
                // Normal speed, 2 lights from right
            }
            EngineerChassisSKD::set_velocity(target_vx, target_vy, yaw_delta);


        } else {
            /// Safe mode
            EngineerChassisLG::set_mode(EngineerChassisLG::FORCE_RELAX_MODE);
        }

        /// Final
        sleep(TIME_MS2I(USER_THREAD_INTERVAL));
    }
}

void UserE::UserActionThread::main() {
    setName("User_Action");

    chEvtRegisterMask(&Remote::s_change_event, &s_change_listener, S_CHANGE_EVENTMASK);
    chEvtRegisterMask(&Remote::mouse_press_event, &mouse_press_listener, MOUSE_PRESS_EVENTMASK);
    chEvtRegisterMask(&Remote::mouse_release_event, &mouse_release_listener, MOUSE_RELEASE_EVENTMASK);
    chEvtRegisterMask(&Remote::key_press_event, &key_press_listener, KEY_PRESS_EVENTMASK);
    chEvtRegisterMask(&Remote::key_release_event, &key_release_listener, KEY_RELEASE_EVENTMASK);

    while (!shouldTerminate()) {

        eventmask_t events = chEvtWaitAny(MOUSE_PRESS_EVENTMASK | MOUSE_RELEASE_EVENTMASK | KEY_PRESS_EVENTMASK);

        /**
         * NOTICE: use flag instead of direct accessing variable in Remote, since event can be trigger by other key, etc
         */

        /// Key Press
        if (events & KEY_PRESS_EVENTMASK) {

            eventflags_t key_flag = chEvtGetAndClearFlags(&key_press_listener);

            if (key_flag & (1U << Remote::KEY_G)) {
                if(Remote::rc.s2 == Remote::S_MIDDLE) {
                    if(!grabber_holded) {
                        EngineerGrabSKD::hold();
                        grabber_holded = true;
                    } else {
                        EngineerGrabSKD::release();
                        grabber_holded = false;
                    }
                }
            }
        }

        // If more event type is added, remember to modify chEvtWaitAny() above
        // Referee client data will be sent by ClientDataSendingThread

    }
}

void UserE::ClientDataSendingThread::main() {

    setName("User_Client");

    while (!shouldTerminate()) {

        /// Send data

        sleep(TIME_MS2I(CLIENT_DATA_SENDING_THREAD_INTERVAL));
    }
}