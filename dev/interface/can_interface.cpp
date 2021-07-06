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
    if (can_driver == &CAND1) {
        setName("CAN1_Monitor");
    } else if (can_driver == &CAND2) {
        setName("CAN2_Monitor");
    } else {
        setName("CAN_Unknown_Monitor");
    }

    event_listener_t el;
    chEvtRegister(&(can_driver->error_event), &el, 0);

    while (!shouldTerminate()) {

        if (waitAnyEventTimeout(ALL_EVENTS, TIME_MS2I(10000)) == 0) {
//            Shell::printf("--- End of error in CAN in last 10 s ---" SHELL_NEWLINE_STR);
            continue;
        }

        eventflags_t flags = chEvtGetAndClearFlags(&el);
//        LOG_ERR("CAN Error %u", flags);
        last_error_time = SYSTIME;
    }

    chEvtUnregister(&(can_driver->error_event), &el);
}

#endif


void CANInterface::start(tprio_t rx_thread_prio, tprio_t tx_thread_prio) {
    canStart(can_driver, &can_cfg);
#if (CAN_INTERFACE_ENABLE_ERROR_FEEDBACK_THREAD == TRUE)
    errorFeedbackThread.can_driver = can_driver;
    errorFeedbackThread.start(LOWPRIO);
#endif
    currentSendThread.can_driver = can_driver;
    currentSendThread.start(tx_thread_prio);

    processFeedbackThread.can_driver = can_driver;
    processFeedbackThread.start(rx_thread_prio);
}

float CANInterface::motor_feedback_t::accumulated_angle() {
    return actual_angle + (float)round_count * 360.0f;
}

void CANInterface::motor_feedback_t::reset_front_angle() {
    actual_angle = 0.0f;
    round_count = 0;
}

void CANInterface::set_target_current(unsigned id, int target_current) {
    currentSendThread.target_current[id] = target_current;
}

void CANInterface::set_motor_type(unsigned int id, motor_type_t motor_type_) {
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

CANInterface::cap_feedback_t *CANInterface::get_cap_feedback_address() {
    return &processFeedbackThread.capfeedback;
}

float *CANInterface::get_lidar_feedback_address() {
    return &processFeedbackThread.lidar_dist;
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
        setName("CAN1_RX");
    } else if (can_driver == &CAND2) {
        setName("CAN2_RX");
    } else {
        setName("CAN_Unknown_RX");
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

            if(rxmsg.SID != 0x211 && rxmsg.SID != 0x003) {
                uint16_t new_actual_angle_raw = (rxmsg.data8[0] << 8 | rxmsg.data8[1]);

                // Check whether this new raw angle is valid
                if (new_actual_angle_raw > 8191) return;

                // pointer fb points to the corresponding motor feedback structure defined in the processFeedbackThread
                motor_feedback_t *fb;
                fb = &feedback[rxmsg.SID - 0x201];

                /// Calculate the angle movement in raw data
                // KEY IDEA: add the change of angle to actual angle
                // We assume that the absolute value of the angle movement is smaller than 180 degrees (4096 of raw data)
                int angle_movement = (int) new_actual_angle_raw - (int) fb->last_angle_raw;

                // Store new_actual_angle_raw for calculation of angle_movement next time
                fb->last_angle_raw = new_actual_angle_raw;

                /// If angle_movement is too extreme between two samples, we grant that it's caused by moving over the 0 (8192) point
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
                        fb->actual_angle += ((float) angle_movement * 0.043945312f);  // * 360 / 8192

                        // rpm -> degree/s
                        fb->actual_velocity = (float)((int16_t) (rxmsg.data8[2] << 8 | rxmsg.data8[3])) * 6.0f;  // 360 / 60

                        fb->actual_current = (int16_t) (rxmsg.data8[4] << 8 | rxmsg.data8[5]);

                        break;

                    case GM6020_HEROH:

                        // raw -> degree
                        fb->actual_angle += ((float)angle_movement * 0.03515625f);  // * 360 / 8192 * deceleration ratio.

                        // rpm -> degree/s
                        fb->actual_velocity = (float)(int16_t)(rxmsg.data8[2] << 8 | rxmsg.data8[3]) * 6.0f * 0.8f;  // 360 / 60

                        fb->actual_current = (int16_t) (rxmsg.data8[4] << 8 | rxmsg.data8[5]);

                        break;

                    case GM3510:  // GM3510 deceleration ratio = 1

                        // raw -> degree
                        fb->actual_angle += angle_movement * 0.043945312f;  // * 360 / 8192

                        fb->actual_velocity = 0;  // no velocity feedback available

                        fb->actual_current = 0;  // no current feedback available

                        break;

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

            } else if (rxmsg.SID == 0x211){

                capfeedback.input_voltage = rxmsg.data16[0] / 100.0f;
                capfeedback.capacitor_voltage = rxmsg.data16[1] / 100.0f;
                capfeedback.input_current = rxmsg.data16[2] / 100.0f;
                capfeedback.output_power = rxmsg.data16[3] / 100.0f;

            } else if (rxmsg.SID == 0x003) {

                lidar_dist = (float) (rxmsg.data8[1] << 8 | rxmsg.data8[0]);
            }
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

bool CANInterface::send_cap_msg(const CANTxFrame *txmsg) {
    currentSendThread.cap_send(txmsg);
    return true;
}

void CANInterface::CurrentSendThread::cap_send(const CANTxFrame *txmsg) {
    capMsg = txmsg;
    capMsgSent = false;
}

void CANInterface::CurrentSendThread::main() {
    if (can_driver == &CAND1) {
        setName("CAN1_TX");
    } else if (can_driver == &CAND2) {
        setName("CAN2_TX");
    } else {
        setName("CAN_Unknown_TX");
    }
    while(!shouldTerminate()) {
        CANTxFrame low_tx_frame;
        CANTxFrame high_tx_frame;

        low_tx_frame.IDE = high_tx_frame.IDE = CAN_IDE_STD;

        low_tx_frame.SID = 0x200;
        high_tx_frame.SID = 0x1FF;

        low_tx_frame.RTR = high_tx_frame.RTR = CAN_RTR_DATA;
        low_tx_frame.DLC = high_tx_frame.DLC = 0x008;

        for (int i = 0; i < 4; i++) {
            if (motorType[i] != RM6623) {
                low_tx_frame.data8[i * 2] = (uint8_t) (target_current[i] >> 8);
                low_tx_frame.data8[i * 2 + 1] = (uint8_t) target_current [i];
            } else {
                low_tx_frame.data8[i * 2] = (uint8_t) (-target_current[i] >> 8);
                low_tx_frame.data8[i * 2 + 1] = (uint8_t) -target_current [i];
            }
        }
        for (int i = 4; i < 8; i++) {
            if (motorType[i] != RM6623) {
                high_tx_frame.data8[i * 2 - 8] = (uint8_t) (target_current[i] >> 8);
                high_tx_frame.data8[i * 2 + 1 - 8] = (uint8_t) target_current [i];
            } else {
                high_tx_frame.data8[i * 2 - 8] = (uint8_t) (-target_current[i] >> 8);
                high_tx_frame.data8[i * 2 + 1 - 8] = (uint8_t) -target_current [i];
            }
        }
        send_msg(&low_tx_frame);
        send_msg(&high_tx_frame);
        if(!capMsgSent) {
            capMsgSent = true;
            send_msg(capMsg);
        }
        sleep(TIME_MS2I(SEND_THREAD_INTERVAL));
    }
}

/** @} */