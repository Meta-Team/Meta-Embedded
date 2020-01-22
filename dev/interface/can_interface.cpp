//
// Created by liuzikai on 2018-12-29.
//


/**
 * @file    can_interface.cpp
 * @brief   CAN interface to receive, distribute and send message.
 *
 * @addtogroup CANInterface
 * @{
 */

#include "can_interface.h"

#if (CAN_INTERFACE_ENABLE_ERROR_FEEDBACK_THREAD == TRUE)

void CANInterface::ErrorFeedbackThread::main() {
    setName("CAN_Err_FB");

    event_listener_t el;
    chEvtRegister(&(can_driver->error_event), &el, 0);

    while (!shouldTerminate()) {

        if (waitAnyEventTimeout(ALL_EVENTS, TIME_MS2I(10000)) == 0) {
//            Shell::printf("--- End of error in CAN in last 10 s ---" SHELL_NEWLINE_STR);
            continue;
        }

        eventflags_t flags = chEvtGetAndClearFlags(&el);
        LOG_ERR("CAN Error %u", flags);
        last_error_time = SYSTIME;
    }

    chEvtUnregister(&(can_driver->error_event), &el);
}

#endif


void CANInterface::start(tprio_t feedback_prio, tprio_t send_current_prio) {
    canStart(can_driver, &can_cfg);
#if (CAN_INTERFACE_ENABLE_ERROR_FEEDBACK_THREAD == TRUE)
    errorFeedbackThread.can_driver = can_driver;
    errorFeedbackThread.start(LOWPRIO);
#endif
    currentSendThread.can_driver = can_driver;
    currentSendThread.start(send_current_prio);

    processFeedbackThread.can_driver = can_driver;
    processFeedbackThread.start(feedback_prio);
}

bool CANInterface::register_callback(uint32_t sid_lower_bound, uint32_t sid_upper_bound,
                                     CANInterface::can_callback_func callback_func) {
    if (callback_list_count >= MAXIMUM_REGISTRATION_COUNT) return false;
    callback_list[callback_list_count++] = {sid_lower_bound, sid_upper_bound, callback_func};
    return true;
}

float CANInterface::motor_feedback_t::accumulated_angle() {
    return actual_angle + round_count * 360.0f;
}

void CANInterface::motor_feedback_t::reset_front_angle() {
    actual_angle = 0.0f;
    round_count = 0;
}

void CANInterface::set_target_current(unsigned id, int target_current) {
    currentSendThread.target_current[id] = target_current;
}

void CANInterface::set_motor_type(unsigned int id, motor_type_t motor_type_) {
    motor_type_list[id] = motor_type_;
    currentSendThread.motorType[id] = motor_type_;
    processFeedbackThread.feedback[id].type = motor_type_;
}

int CANInterface::echo_target_current(unsigned id) {
    return currentSendThread.target_current[id];
}

int *CANInterface::get_target_current_address(unsigned int id) {
    return &currentSendThread.target_current[id];
}

CANInterface::motor_feedback_t *CANInterface::get_feedback_address(unsigned id) {
    return &processFeedbackThread.feedback[id];
}

void CANInterface::ProcessFeedbackThread::main() {
    /**
     * Note that there are two can instances defined in the main function at the very first beginning,
     * so the can1 and can2 has independent thread.
     * The two threads have there own scope of data and two cans will only process their own feedback data;
     * That's why the pointer fb defined below knows which can wire does the motor feedback signal belongs to:
     *      motor_feedback_t *fb;
     *      fb = &feedback[rxmsg.SID - 0x201];
     * Note that this feedback array represents all the motors ever loaded to this can and the maximal number is 8:
     *      motor_feedback_t feedback[MAXIMUM_MOTOR_COUNT + 1];
     */
    if (can_driver == &CAND1) {
        setName("FEEDBACK_CAN1");
    } else if (can_driver == &CAND2) {
        setName("FEEDBACK_CAN2");
    } else {
        setName("CAN_Unknown");
    }

    CANRxFrame rxmsg;
    event_listener_t el;

    // Register an event listener for the rx event source from CANInterface driver
    chEvtRegister(&(can_driver->rxfull_event), &el, 0);

    while (!shouldTerminate()) {

        // Wait until a event occurs
        waitAnyEvent(ALL_EVENTS);

        // Process every received message
        while (canReceive(can_driver, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE) == MSG_OK) {

            uint16_t new_actual_angle_raw = (rxmsg.data8[0] << 8 | rxmsg.data8[1]);

            // Check whether this new raw angle is valid
            if (new_actual_angle_raw > 8191) return;

            motor_feedback_t *fb;
            //pointer fb points to the corresponding motor feedback structure defined in the processFeedbackThread
            fb = &feedback[rxmsg.SID - 0x201];

            /// Calculate the angle movement in raw data
            // KEY IDEA: add the change of angle to actual angle
            // We assume that the absolute value of the angle movement is smaller than 180 degrees (4096 of raw data)
            int angle_movement = (int) new_actual_angle_raw - (int) fb->last_angle_raw;

            // Store new_actual_angle_raw for calculation of angle_movement next time
            fb->last_angle_raw = new_actual_angle_raw;

            /// If angle_movement is too extreme between two samples, we grant that it's caused by moving over the 0(8192) point
            if (angle_movement < -4096) angle_movement += 8192;
            if (angle_movement > 4096) angle_movement -= 8192;

            switch (fb->type) {

                case RM6623:  // RM6623 deceleration ratio = 1

                    // raw -> degree
                    fb->actual_angle += angle_movement * 0.043945312f;  // * 360 / 8192

                    fb->actual_velocity = 0;  // no velocity feedback available

                    fb->actual_current = (int16_t) (rxmsg.data8[2] << 8 | rxmsg.data8[3]);

                    break;

                case M2006:  // M2006 deceleration ratio = 36,

                    // raw -> degree with deceleration ratio
                    fb->actual_angle += angle_movement * 0.001220703f;  // * 360 / 8192 / 36

                    // rpm -> degree/s with deceleration ratio
                    fb->actual_velocity = ((int16_t) (rxmsg.data8[2] << 8 | rxmsg.data8[3])) * 0.166666667f;  // 360 / 60 / 36

                    fb->actual_current = 0;  // no current feedback available

                    break;

                case M3508:  // M3508 deceleration ratio = 3591/187

                    // raw -> degree with deceleration ratio
                    fb->actual_angle += angle_movement * 0.002288436f; // 360 / 8192 / (3591/187)

                    // rpm -> degree/s with deceleration ratio
                    fb->actual_velocity =
                            ((int16_t) (rxmsg.data8[2] << 8 | rxmsg.data8[3])) * 0.312447786f;  // 360 / 60 / (3591/187)

                    fb->actual_current = (int16_t) (rxmsg.data8[4] << 8 | rxmsg.data8[5]);

                    break;

                case GM6020:  // GM6020 deceleration ratio = 1

                    // raw -> degree
                    fb->actual_angle += angle_movement * 0.043945312f;  // * 360 / 8192

                    // rpm -> degree/s
                    fb->actual_velocity = ((int16_t) (rxmsg.data8[2] << 8 | rxmsg.data8[3])) * 6.0f;  // 360 / 60

                    fb->actual_current = (int16_t) (rxmsg.data8[4] << 8 | rxmsg.data8[5]);

                    break;

                case GM3510:  // GM3510 deceleration ratio = 1

                    // raw -> degree
                    fb->actual_angle += angle_movement * 0.043945312f;  // * 360 / 8192

                    fb->actual_velocity = 0;  // no velocity feedback available

                    fb->actual_current = 0;  // no current feedback available

                default:
                    break;
            }

            /// Normalize the angle to [-180, 180]
            // If the actual_angle is greater than 180(-180) then it turns a round in CCW(CW) direction
            if (fb->actual_angle >= 180.0f) {
                fb->actual_angle -= 360.0f;
                fb->round_count++;
            }
            if (fb->actual_angle < -180.0f) {
                fb->actual_angle += 360.0f;
                fb->round_count--;
            }

            fb->last_update_time = SYSTIME;
        }

    }

    chEvtUnregister(&(can_driver->rxfull_event), &el);
}

bool CANInterface::CurrentSendThread::send_msg(const CANTxFrame *txmsg) {
    if (canTransmit(can_driver, CAN_ANY_MAILBOX, txmsg, TIME_MS2I(TRANSMIT_TIMEOUT_MS)) != MSG_OK) {
        // TODO: show debug info for failure
        return false;
    }
    return true;
}

void CANInterface::CurrentSendThread::main() {
    if (can_driver == &CAND1) {
        setName("CURRENT_SEND_CAN1");
    } else if (can_driver == &CAND2) {
        setName("CURRENT_SEND_CAN2");
    } else {
        setName("CAN_Unknown");
    }
    while(!shouldTerminate()) {
        CANTxFrame CAN_LOW_TX_FRAME;
        CANTxFrame CAN_HIGH_TX_FRAME;

        CAN_LOW_TX_FRAME.IDE = CAN_HIGH_TX_FRAME.IDE = CAN_IDE_STD;

        CAN_LOW_TX_FRAME.SID = 0x200;
        CAN_HIGH_TX_FRAME.SID = 0x1FF;

        CAN_LOW_TX_FRAME.RTR = CAN_HIGH_TX_FRAME.RTR = CAN_RTR_DATA;
        CAN_LOW_TX_FRAME.DLC = CAN_HIGH_TX_FRAME.DLC = 0x008;

        for (int i = 0; i < 4; i++) {
            if (motorType[i] != RM6623) {
                CAN_LOW_TX_FRAME.data8[i * 2] = (uint8_t) (target_current[i] >> 8);
                CAN_LOW_TX_FRAME.data8[i * 2 + 1] = (uint8_t) target_current [i];
            } else {
                CAN_LOW_TX_FRAME.data8[i * 2] = (uint8_t) (-target_current[i] >> 8);
                CAN_LOW_TX_FRAME.data8[i * 2 + 1] = (uint8_t) -target_current [i];
            }
        }
        for (int i = 4; i < 8; i++) {
            if (motorType[i] != RM6623) {
                CAN_HIGH_TX_FRAME.data8[i * 2 - 8] = (uint8_t) (target_current[i] >> 8);
                CAN_HIGH_TX_FRAME.data8[i * 2 + 1 - 8] = (uint8_t) target_current [i];
            } else {
                CAN_HIGH_TX_FRAME.data8[i * 2 - 8] = (uint8_t) (-target_current[i] >> 8);
                CAN_HIGH_TX_FRAME.data8[i * 2 + 1 - 8] = (uint8_t) -target_current [i];
            }
        }
        send_msg(&CAN_LOW_TX_FRAME);
        send_msg(&CAN_HIGH_TX_FRAME);
        sleep(TIME_MS2I(SEND_THREAD_INTERVAL));
    }
}

/** @} */