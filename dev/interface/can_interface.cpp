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
        setName("CAN?_Monitor");
    }

    event_listener_t el;
    chEvtRegister(&(can_driver->error_event), &el, 0);

    while (!shouldTerminate()) {

        if (waitAnyEventTimeout(ALL_EVENTS, TIME_MS2I(10000)) == 0) {
//            Shell::printf("--- End of error in CAN in last 10 s ---" SHELL_NEWLINE_STR);
            continue;
        }

//        eventflags_t flags = chEvtGetAndClearFlags(&el);
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
    txThread.can_driver = can_driver;
    txThread.start(tx_thread_prio);

    rxThread.can_driver = can_driver;
    rxThread.start(rx_thread_prio);
}

float CANInterface::motor_feedback_t::accumulated_angle() {
    return (float) accumulated_movement_raw * CAN_INTERFACE_RAW_TO_ANGLE_RATIO / deceleration_ratio;
}

void CANInterface::motor_feedback_t::reset_front_angle() {
    actual_angle = 0.0f;
    accumulated_movement_raw = 0;
}

int *CANInterface::register_target_current_address(unsigned int id, motor_type_t motor_type) {
    unsigned tx_id = 4;
    int *p;
    motor_type_t *m;
    switch (motor_type) {
        case M2006:
        case M3508:
            if (id <= 4 && id >= 1) {
                p = txThread.x200_target_current;
                m = txThread.x200_motorType;
                tx_id = id - 1;
            } else if (id >= 5 && id <= 8) {
                p = txThread.x1ff_target_current;
                m = txThread.x1ff_motorType;
                tx_id = id - 5;
            } else return nullptr;
            break;
        case GM3510:
            if (id >= 1 && id <= 3) {
                p = txThread.x1ff_target_current;
                m = txThread.x1ff_motorType;
                tx_id = id - 1;
            } else return nullptr;
            break;
        case GM6020:
            if (id <= 4 && id >= 1) {
                p = txThread.x1ff_target_current;
                m = txThread.x1ff_motorType;
                tx_id = id - 1;
            }
#if CAN_INTERFACE_ENABLE_X2FF_FRAME
            else if (id >= 5 && id <= 7) {
                p = txThread.x2ff_target_current;
                m = txThread.x2ff_motorType;
                tx_id = id - 5;
            }
#endif
            else return nullptr;
            break;
        default:
            return nullptr;
    }
    m[tx_id] = motor_type;
    return p + tx_id;
}

CANInterface::motor_feedback_t *CANInterface::register_feedback_address(unsigned id, motor_type_t motor_type, float deceleration_ratio) {
    unsigned rx_id = MAXIMUM_MOTOR_COUNT;
    switch (motor_type) {
        case M3508:
        case M2006:
            if (id <= 8 && id >= 1) rx_id = id - 1;
            break;
        case GM3510:
            if (id <= 3 && id >= 1) rx_id = id + 4 - 1;
            break;
        case GM6020:
            if (id <= 7 && id >= 1) rx_id = id + 4 - 1;
            break;
        case NONE_MOTOR:
        default:
            break;
    }
    rxThread.feedback[rx_id].type = motor_type;
    rxThread.feedback[rx_id].deceleration_ratio = deceleration_ratio;
    return &rxThread.feedback[rx_id];
}

CANInterface::cap_feedback_t *CANInterface::get_cap_feedback_address() {
    return &rxThread.capfeedback;
}

float *CANInterface::get_lidar_feedback_address() {
    return &rxThread.lidar_dist;
}

void CANInterface::RxThread::main() {
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
        setName("CAN?_RX");
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

            if(rxmsg.SID <= 0x20B && rxmsg.SID >= 0x201) {
                uint16_t new_actual_angle_raw = (rxmsg.data8[0] << 8 | rxmsg.data8[1]);

                // Check whether this new raw angle is valid
                if (new_actual_angle_raw > 8191) return;

                // pointer fb points to the corresponding motor feedback structure defined in the processFeedbackThread
                motor_feedback_t *fb;
                fb = &feedback[rxmsg.SID - 0x201];

                if (fb->type == NONE_MOTOR) continue;

                fb->sid = rxmsg.SID;

                /// Calculate the angle movement in raw data
                // KEY IDEA: add the change of angle to actual angle
                // We assume that the absolute value of the angle movement is smaller than 180 degrees (4096 of raw data)
                int angle_movement = (int) new_actual_angle_raw - (int) fb->last_angle_raw;

                // Store new_actual_angle_raw for calculation of angle_movement next time
                fb->last_angle_raw = new_actual_angle_raw;

                /// If angle_movement is too extreme between two samples, we grant that it's caused by moving over the 0 (8192) point
                if (angle_movement < -4096) angle_movement += 8192;
                if (angle_movement > 4096) angle_movement -= 8192;

                fb->accumulated_movement_raw += angle_movement;

                // raw -> degree
                fb->actual_angle += (float) angle_movement * CAN_INTERFACE_RAW_TO_ANGLE_RATIO / fb->deceleration_ratio;

                switch (fb->type) {

                    case M2006:  // M2006 deceleration ratio = 36,

                        fb->actual_velocity = (float) ((int16_t) (rxmsg.data8[2] << 8 | rxmsg.data8[3])) * CAN_INTERFACE_RPM_TO_DPS / fb->deceleration_ratio;

                        fb->actual_current = 0;  // no current feedback available

                        break;

                    case M3508:  // M3508 deceleration ratio = 3591/187

                        fb->actual_velocity = (float) ((int16_t) (rxmsg.data8[2] << 8 | rxmsg.data8[3])) * CAN_INTERFACE_RPM_TO_DPS / fb->deceleration_ratio;

                        fb->actual_current = (int16_t) (rxmsg.data8[4] << 8 | rxmsg.data8[5]);

                        break;

                    case GM6020:  // GM6020 deceleration ratio = 1

                        // rpm -> degree/s
                        fb->actual_velocity = (float) ((int16_t) (rxmsg.data8[2] << 8 | rxmsg.data8[3])) * CAN_INTERFACE_RPM_TO_DPS;

                        fb->actual_current = (int16_t) (rxmsg.data8[4] << 8 | rxmsg.data8[5]);

                        break;

                    case GM3510:  // GM3510 deceleration ratio = 1

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
                }
                if (fb->actual_angle < -180.0f) {
                    fb->actual_angle += 360.0f;
                }

                fb->last_update_time = SYSTIME;

            } else if (rxmsg.SID == 0x211) {

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

bool CANInterface::TxThread::send_msg(const CANTxFrame *txmsg) {
    if (canTransmitTimeout(can_driver, CAN_ANY_MAILBOX, txmsg, TIME_MS2I(TRANSMIT_TIMEOUT_MS)) != MSG_OK) {
        /*if (can_driver == &CAND1) {
            LOG_ERR("CAN1 TX timed out");
        } else if (can_driver == &CAND2) {
            LOG_ERR("CAN2 TX timed out");
        } else {
            LOG_ERR("CAN? TX timed out");
        }*/
        return false;
    }
    return true;
}

bool CANInterface::send_cap_msg(const CANTxFrame *txmsg) {
    txThread.cap_send(txmsg);
    return true;
}

void CANInterface::TxThread::cap_send(const CANTxFrame *txmsg) {
    capMsg = txmsg;
    capMsgSent = false;
}

void CANInterface::TxThread::main() {
    if (can_driver == &CAND1) {
        setName("CAN1_TX");
    } else if (can_driver == &CAND2) {
        setName("CAN2_TX");
    } else {
        setName("CAN?_TX");
    }
    while (!shouldTerminate()) {
        CANTxFrame x200_tx_frame;
        CANTxFrame x1ff_tx_frame;
        x200_tx_frame.IDE = x1ff_tx_frame.IDE = CAN_IDE_STD;
        x200_tx_frame.RTR = x1ff_tx_frame.RTR = CAN_RTR_DATA;
        x200_tx_frame.DLC = x1ff_tx_frame.DLC = 0x008;
        x200_tx_frame.SID = 0x200;
        x1ff_tx_frame.SID = 0x1FF;

#if CAN_INTERFACE_ENABLE_X2FF_FRAME
        CANTxFrame x2ff_tx_frame;
        x2ff_tx_frame.IDE = CAN_IDE_STD;
        x2ff_tx_frame.SID = 0x2FF;
        x2ff_tx_frame.RTR = CAN_RTR_DATA;
        x2ff_tx_frame.DLC = 0x008;
#endif

        for (int i = 0; i < 4; i++) {
            x200_tx_frame.data8[i * 2] = (uint8_t) (x200_target_current[i] >> 8);
            x200_tx_frame.data8[i * 2 + 1] = (uint8_t) x200_target_current[i];

            x1ff_tx_frame.data8[i * 2] = (uint8_t) (x1ff_target_current[i] >> 8);
            x1ff_tx_frame.data8[i * 2 + 1] = (uint8_t) x1ff_target_current[i];

#if CAN_INTERFACE_ENABLE_X2FF_FRAME
            x2ff_tx_frame.data8[i * 2] = (uint8_t) (x2ff_target_current[i] >> 8);
            x2ff_tx_frame.data8[i * 2 + 1] = (uint8_t) x2ff_target_current [i];
#endif

        }
        send_msg(&x200_tx_frame);
        send_msg(&x1ff_tx_frame);
#if CAN_INTERFACE_ENABLE_X2FF_FRAME
        send_msg(&x2ff_tx_frame);
#endif
        if (!capMsgSent) {
            capMsgSent = true;
            send_msg(capMsg);
        }
        sleep(TIME_MS2I(SEND_THREAD_INTERVAL));
    }
}

/** @} */